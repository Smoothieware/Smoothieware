#!/usr/bin/perl

use strict;

while (defined $ARGV[0]) {
	my $s1 = 0;
	my $s2 = 0;
	my $str = shift @ARGV;

	for (split //, $str) {
		$s1 = ($s1 + (ord $_)) % 255;
		$s2 = ($s2 + $s1) % 255;
	};
	printf "%s:\t%d\n", $str, ($s2<<8) + $s1;
}
