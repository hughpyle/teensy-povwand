/*
  Arduino sketch for Teensy 3.0 levitation wand

	- Run a main interrupt timer (PIT1) at ~10kHz.
	- Wire the analog converter sampling to the PIT1 timer.
	- At about ~200Hz, refresh the LEDs.
		- Something related to the ambient audio, beats, etc.
		- Some background colors, etc.
		- Some text for persistence-of-vision display.
		
  
*/

#include <avr/pgmspace.h>

#include "font_c64d.h"    // 16-pixel bitmap fonts
#define FONTDATA font_c64d

#include <FastSPI_LED.h>
#include <fix_fft.h>      // 8-bit FFT


// Clock frequency (Hz) for the CPU.  96M or 48M or 24MHz
#define CLOCKSPEED 96000000

// Sampling frequency (Hz) for the ADC
#define FREQ 9375

// Number of ticks (ADC samples) before we do a "frame" (LED update).  Goal: refresh ~240Hz
#define TICKSPERFRAME 40

// Number of ticks (ADC samples) before we do a "main loop" (FFT update).  This should be at leat BUFSIZE.
#define TICKSPERLOOP 128

// Circular buffer for audio samples, and buffer for the FFT
#define BUFSIZE 128
// Buffer size is 2^BUFPOW2
#define BUFPOW2 7

// The FFT is symmetric about Nyquist, so we only care about half
#define DATASIZE (BUFSIZE/2)

// Analog input is AIN1 (Teensy3 pin 14, next to LED)
#define ANALOGPIN A1

#define DEBUGPIN 3

// Number of LEDs in the strip
#define NUM_LEDS 34

// Pin for the on-board LED (only for debug - with the string of leds this pin is used for SCK)
#define LEDPIN 13


// Different mfr strips might have RGB in different sequences.  Mine came from Adafruit
//struct CRGB { unsigned char b; unsigned char r; unsigned char g; };
struct CRGB { unsigned char r; unsigned char g; unsigned char b; };
struct CRGB *leds;

// Audio statistics
int audioAmplitude = 0;
float avgAudioAmplitude = 0;

float audioFreqStdev = 0;
float avgAudioFreqStdev = 0;

// RGB colorwheels
float valRx = 1, valRy = 0;
float valGx = 1, valGy = 0;
float valBx = 1, valBy = 0;
float dR = 0.02;
float dG = 0.017;
float dB = 0.012;

// Circular buffer to store analog samples,
// and a copy buffer for the FFT to work in.
static volatile uint16_t head = 0, tail = 0;
static volatile int16_t buffer[BUFSIZE];
static          int16_t fftbuf[BUFSIZE];  // buffer of real
static          int16_t fftbif[BUFSIZE];  // buffer of im, nulls
static          int32_t ledbuf[NUM_LEDS];
static volatile uint32_t peakVal = 0;
static volatile boolean wasPeak = false, isPeak = false;


// -----
// Initialize the timer interrupt
// http://forum.pjrc.com/threads/14-Teensy-3-0-and-interrupts

// Constants for bitvalues within the TCTRL1 register
#define TIE 2
#define TEN 1
volatile int ledVal;
volatile uint32_t nFrameTicks;
volatile uint32_t nLoopTicks;
volatile boolean  doMainLoop;

void setup_timer()
{
  nFrameTicks = 0;
  nLoopTicks = 0;
  doMainLoop = false;
  
  cli();
 
  // Set the ADC to trigger on PIT0 interrupt timer
  // page 235 
  // 0100 PIT trigger 0
  // 0101 PIT trigger 1
  // 0110 PIT trigger 2
  // 0111 PIT trigger 3
  SIM_SOPT7 = ( SIM_SOPT7 & 0xFFF0 ) + 5;

  // Teensy 3.0 
  SIM_SCGC6 |= SIM_SCGC6_PIT; // Activates the clock for PIT
  // Turn on PIT
  PIT_MCR = 0x00;
  // Set the period of the timer
  PIT_LDVAL1 = CLOCKSPEED/FREQ;
  // Enable interrupts on timer1
  PIT_TCTRL1 = TIE;
  // Start the timer
  PIT_TCTRL1 |= TEN;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1); // Another step to enable PIT channel 1 interrupts
  sei();
}

void pit1_isr(void)
{
  PIT_TFLG1 = 1;  
  nFrameTicks++;
  nLoopTicks++;
  
  readADC();

  if( nFrameTicks>=TICKSPERFRAME )
  {
    nFrameTicks = 0;
    doFrame();
  }
  
  if( nLoopTicks>=TICKSPERLOOP )
  {
    doMainLoop = true;
    nLoopTicks = 0;
  }
}


// TODO
// Called within ISR
void doFrame()
{
  writeLEDs();
}

// -----
// Write LEDs

uint16_t fftBin( int n )
{
  // Linearly divide the FFT bins between the LEDs
  return (n * DATASIZE) / NUM_LEDS;
}

char* text = "HAPPY NEW YEAR 2013 ";  // terminate with a space
char* ptext = text;
uint16_t scanline = 0;
uint16_t scandata = 0;
boolean  showtext = false;

void writeLEDs()
{
  int i;
  long n;
  long ledtot = 0;
  int vR;
  int vG;
  int vB;
  int brightness = 25;

  // Override the default brightness when loud sounds
  int aa = (int)avgAudioAmplitude >> 2;
  if( aa > brightness )
    brightness = max(aa,63);
    
  // When we hit a peak, show a new word of the text
  if( isPeak )
  {
    isPeak = false;
    showtext = true;
  }
  
  if( showtext )
  {
    // Calculate font scanline data
    char c = *ptext - ' ';
    uint16_t t = 0;
    if( c<95 )
    {
      scandata = FONTDATA[c][scanline];
    }
    if( scanline==0 )
    {
      Serial.println(*ptext);
    }
    scanline++;
    if( scanline>=16 )
    {
      if( c==0 )        // This is the last scanline of a space.  Stop until another noise peak.
      {
        showtext = false;
      }
      scanline = 0;     // Start the next character
      ptext++;
    }
    if( *ptext=='\0' )
    {
      ptext = text;
    }
  }
  
  
  // Update the three phase colorwheels
  valRx = valRx + (dR * valRy);  valRy = valRy - (dR * valRx);
  valGx = valGx + (dG * valGy);  valGy = valGy - (dG * valGx);
  valBx = valBx + (dB * valBy);  valBy = valBy - (dB * valBx);
  vR = brightness * (1+valRx);
  vG = brightness * (1+valGx);
  vB = brightness * (1+valBx);
  
  // Read the current FFT data
  for(i = 0 ; i < NUM_LEDS; i++)
  {
    // Average the fft bins for this LED
    int32_t a = 0;
    int m = 0;
    for( int n=fftBin(i); n<fftBin(i+1); n++ )
    {
      a += fftbuf[n];
      m++;
    }
    n = a / m;
    ledbuf[i] = n;
    ledtot += ledbuf[i];
  }

//  Serial.print("\x1b[1;1f");

  // Write the results into the LED strip
  for(i = 0 ; i < NUM_LEDS; i++)
  {
    long a = ledbuf[i] / 256; // ( 255 * ledbuf[i]) / ledtot;
//    if( a<0 ) a=-a;

//        Serial.print(i,DEC);
//        Serial.print(" ");
//        Serial.print(a,DEC);
//        for( int k=0; k<a/4; k++ )  Serial.print("*");
//        Serial.println("    ");
        
    uint16_t x = ( scandata & 1 )>0 ? 100 : 0;
    scandata = scandata >> 1;

    leds[i].r = (uint16_t)vR | x;
    leds[i].g = (uint16_t)vG | x;
    leds[i].b = (uint16_t)vB | x;
  }
  
  FastSPI_LED.show();
}



// -----

// Read one sample from the ADC into the buffer
void readADC()
{
  uint16_t h;
  int16_t val;
  
  val = analogRead(ANALOGPIN);

  // Write new data into the buffer.  Always wrap over the tail (don't wait for readers).        
  h = head + 1;
  if (h >= BUFSIZE) h = 0;
  buffer[h] = val;
  head = h;
}

// ----
// Perform FFT of the audio samples, and other useful statistics.
void doFFT()
{
  uint16_t n;
  int32_t bmean;
  int16_t bmin = 32767;
  int16_t bmax = -32767;
  int32_t fmean;
  int16_t fmin = 32767;
  int16_t fmax = -32767;
  float gmean;
  uint16_t h;
  
  // Find mean, max, min of the audio buffer
  bmean = 0;
  for( n=0; n<BUFSIZE; n++ )
  {
    int16_t b = buffer[n];
    bmean += b;
    if( b > bmax ) bmax = b;
    if( b < bmin ) bmin = b;
  }
  bmean = bmean / BUFSIZE;
  audioAmplitude = bmax - bmin;  // theoretical range 0-1023.  Actually max ~600
  
  // Moving average
  if( audioAmplitude > avgAudioAmplitude )
  {
    avgAudioAmplitude = audioAmplitude;
    // Serial.println( avgAudioAmplitude, DEC );
    if( avgAudioAmplitude > 50 )
    {
      wasPeak = isPeak;
      isPeak = true;
    }
  }
  else
    avgAudioAmplitude = ( (int32_t)19 * avgAudioAmplitude + audioAmplitude ) / 20;
  
  // Copy into the FFT buffer (aligned to "head" so we can window correctly).
  // Subtract the mean value from all samples, to remove DC.
  // Then normalize to half-full-scale.
  h = head;
  for( n=0; n<BUFSIZE; n++ )
  {
    fftbuf[n] = ( (int32_t)( buffer[h] - bmean ) ); // * 16384 ); // / avgAudioAmplitude;
    fftbif[n] = 0;
    h++;
    if( h>=BUFSIZE ) h=0;
  }

  // Window the buffer
  fix_window(fftbuf, BUFPOW2);

  // FFT
  fix_fft(fftbuf, fftbif, BUFPOW2, 0);

  // Convert each value to magnitude
  for( n=0; n<DATASIZE; n++ )
  {
    uint32_t i = sqrt( (uint32_t)(fftbuf[n]*fftbuf[n]) + (uint32_t)(fftbif[n]*fftbif[n]) );
    if(i>32767) i=32767;
    fftbuf[n] = i;
  }
  
  // Linearize the spectrum
  // 32 16 8 4 2 1 2 4 8 16 32
  
  // Find the standard deviation of the spectrum.  High sdev <- pure tones (?)
  fmean = 0;
  for( n=0; n<DATASIZE; n++ )
  {
    int16_t f = fftbuf[n];
    fmean += f;
    if( f > fmax ) fmax = f;
    if( f < fmin ) fmin = f;
  }
  fmean = fmean / DATASIZE;
  gmean = 0;
  for( n=0; n<DATASIZE; n++ )
  {
    int16_t g = fftbuf[n] - fmean;
    gmean += g * g;
  }
  gmean = gmean / DATASIZE;
  audioFreqStdev = sqrt(gmean);

  // Moving average
  avgAudioFreqStdev = ( (int32_t)9 * avgAudioFreqStdev + audioFreqStdev ) / 10;
  
  /*
  Serial.print( bmin, DEC );
  Serial.print( " " );
  Serial.print( bmax, DEC );
  Serial.print( " " );
  Serial.print( audioAmplitude, DEC );
  Serial.print( " " );
  Serial.print( fmin, DEC );
  Serial.print( " " );
  Serial.print( fmax, DEC );
  Serial.print( " " );
  Serial.print( audioFreqStdev, DEC );
  Serial.println( "         " );
  */
  
  if( digitalRead(DEBUGPIN)==HIGH )
  {
  for( n=0; n<BUFSIZE; n++ )
  {
    Serial.println( buffer[n], DEC );
  }
  }

}




void setup()
{
  Serial.begin(115200);              // baud rate is ignored with Teensy USB ACM i/o

  // DEBUG
//  pinMode(LEDPIN, OUTPUT);
//  ledVal = 0;
//  digitalWrite(LEDPIN, ledVal);
  // END DEBUG

  FastSPI_LED.setLeds(NUM_LEDS);
  FastSPI_LED.setChipset(CFastSPI_LED::SPI_LPD8806);
  FastSPI_LED.init();
  FastSPI_LED.start();
  leds = (struct CRGB*)FastSPI_LED.getRGBData(); 

  pinMode(ANALOGPIN, INPUT);
  pinMode(DEBUGPIN, INPUT);

  analogReference(EXTERNAL);         // Internal reference is 1.2v, but our data goes to 3.3v
  analogReadResolution(10);          // Teensy 3.0: set ADC resolution to this many bits (more is unnecessary)
  analogReadAveraging(4);            // Teensy 3.0: how many samples to average
  
  // Read once, to calibrate
  analogRead(ANALOGPIN);
  
  delay(5000);   // wait for slow human to get serial capture running
  Serial.println("# hello");

  setup_timer();
}



void loop()
{
  if( !doMainLoop ) return;
  doMainLoop = false;
  
  // DEBUG
// digitalWrite(LEDPIN, ledVal ^= 1);

  doFFT();
}

