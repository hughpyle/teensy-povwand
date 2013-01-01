#!/usr/bin/perl

# bsd2c.pl - exports a C array for vertical scanline dotmatrix displays from BDF fonts
# Derived / inspired from Markus Kuhn <http://www.cl.cam.ac.uk/~mgk25/> ucs2any.pl

#use strict;
use Data::Dumper;

sub bin { return unpack("N", pack("B32", substr("0" x 32 . shift, -32))); }
sub dec2hex { my $hex = unpack("H8", pack("N", shift)); my $zeros = "0" x (8 - int(shift)); $hex =~ s/$zeros//; return $hex; }
sub dec2bin { my $bin = unpack("B32", pack("N", shift)); my $zeros = "0" x (32 - int(shift)); $bin =~ s/$zeros//; return $bin; }

# byte lenght of character
sub bytes
{
    my ($cols, $rows) = @_;
    return 1 + (($dir) ? int(($rows-1) / 8) : int(($cols-1) / 8));
}

# open and read source file
$fsource = $ARGV[0];
open(FSOURCE,  "<$fsource")  || die ("Can't read file '$fsource': $!\n");

# read header
$properties = 0;
$default_char = 0;
while (<FSOURCE>) {
    last if /^CHARS\s/;
    if (/^STARTFONT/) {
        $startfont = $_;
    } elsif (/^FONT\s+(.*-([^-]*-\S*))\s*$/) {
        $font = $1;
    }
}

die ("No STARTFONT line found in '$fsource'!\n") unless $startfont;

# read characters
while (<FSOURCE>) {
    if (/^STARTCHAR/) {
	$sc = $_;
	$code = -1;
    } elsif (/^ENCODING\s+(-?\d+)/) {
        $code = $1;
	$startchar{$code} = $sc;
	$char{$code} = "";
    } elsif (/^ENDFONT$/) {
	$code = -1;
	$sc = "STARTCHAR ???\n";
    } else {
        $char{$code} .= $_;
        if (/^ENDCHAR$/) {
            $code = -1;
	    $sc = "STARTCHAR ???\n";
        }
    }
    next if ($code < 0x20);
    next if ($code > 0x7f);
}
close FSOURCE;

my $text;

# loop thru ASCII printable (0x20-0x7f) characters
for (my $i = 0x20; $i < 0x7f; $i++)
{
    # detect width and height
    $char{$i} =~ m/BBX (\d+) (\d+)/;
    $w = $1;
    $h = $2;
    $len = $w;
    
    # extract character bitmap
    @matrix = ();
    $char{$i} =~ m/BITMAP(.+?)ENDCHAR/gms;
    $bitmap = $1;
    while ($bitmap =~ m/([0-9A-Fa-f]+?)\n/gms)
    {
	$line = $1;
	push @matrix, hex($line);
    }
    #print Dumper(@matrix);
    
    # init bit matrix 
    @bitmatrix = ();
    for (my $y = 0; $y < $h; $y++) {
        for (my $x = 0; $x < $w; $x++) {
            $bitmatrix[$y][$x] = 0;
        }
    }
    $bytes = bytes($w, $h);
    for (my $y = 0; $y < $h; $y++)
    {
        @{$bitmatrix[$y]} = split('', dec2bin($matrix[$y], $bytes * 8));
    }
    #print Dumper(@bitmatrix);
    
    # rotate character bitmap for vertical scanline
    @matrix = ();
    for (my $x = 0; $x < $w; $x++)
    {
        my @tmp = ();
        for (my $y = 0; $y < $h; $y++) {
            push @tmp, $bitmatrix[$y][$x];
        }
        $matrix[$x] = bin(join("", @tmp));
    }
    #print Dumper(@matrix);
    
    # print C array of bitmap
    $text .= "{";
    $bytes = bytes($h, $w);
    for (my $c = 0; $c < $w; $c++) {
        my $hex = "0x" . dec2hex($matrix[$c], $bytes * 2);
        $text .= "$hex" if ($c == $w-1);
        $text .= "$hex," if ($c < $w-1);
    }
    $chr = chr($i);
    $chr = '"\"' if ($i == 92);
    $text .= "}, // $chr \n";
}

print "// created with bdf2pl, see https://code.google.com/p/dotmatrix-editor/source/browse/ \n";
print "// ", $font, "\n";
$a_name = lc($fsource);
$a_name =~ s/.*\///;
$a_name =~ s/\.bdf//;
$a_type = sprintf("uint%d_t PROGMEM", $bytes * 8) if !$a_type;
$text = "$a_type font_${a_name}[95][$len] = {\n$text};\n";
print "$text\n";

delete $char{-1};
