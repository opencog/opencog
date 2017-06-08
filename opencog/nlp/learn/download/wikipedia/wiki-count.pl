#! /usr/bin/env perl
#
#  wiki-count.pl
#
# Script to count how many articles there are in a wikipedia xml dump.
# 
# Example usage:
# cat simplewiki-20080629.xml.bz2 | bunzip2 | ./wiki-count.pl
# 
# Copyright (c) 2008, 2013 Linas Vepstas <linas@linas.org>
#
#--------------------------------------------------------------------
# Need to specify the binmodes, in order for \w to match utf8 chars
use utf8;
binmode STDIN, ':encoding(UTF-8)'; 
binmode STDOUT, ':encoding(UTF-8)';

$lcnt = 0;
$cnt = 0;
$last = 0;

while (<>)
{
	$lcnt += 1;
	if (/<title>(.+?)<\/title>/) {
		$cnt += 1;
	}

	if (0 == $cnt % 100000 && $cnt != $last) {
		print "So far up to $cnt ...\n";
		$last = $cnt;
	}
}

print "Counted $lcnt lines in $cnt pages\n";
