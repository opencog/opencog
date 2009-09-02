#! /usr/bin/env perl
#
# sentence-split.pl
#
# Take as input a file containing multiple sentences expressed
# as opencog scheme. Split it into multiple files, with each file
# containing one sentence.
#
# Usage:
#    cat somefile.scm | ./sentence-split.pl /some/file-prefix-
#
# will take each sentence in "somefile.scm", and put it in a
# sequentially-numered file whose name starts with
# "/some/file-prefix-"
#
#  Linas Vepstas April 2009
#

use strict;

if ($#ARGV < 0)
{
	die "Error: Need to specify fiel prefix\n";
}

my $file_prefix = $ARGV[0];
my $count = 0;

while(<STDIN>)
{
	if (/SENTENCE:/)
	{
		close FILE;
		$count ++;
		my $filename = $file_prefix . $count . ".scm";
		print "Processing $filename\n";
		open FILE, "> $filename" or die "Can't open $filename : $!";
	}
	print FILE $_;
}
close FILE;
