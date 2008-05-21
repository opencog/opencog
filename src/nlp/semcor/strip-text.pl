#! /usr/bin/env perl
#
# This file just stripes out the sentences from the semcor files.

# Loop -- read from stdin, and put quotes around the attributes.
$leading_quote = 0;
while(<>)
{
	chop;
	s/<[\/\:\$\'\w\s=-]+>//g;  # strip out all xml markup

	s/_/ /g;  # replace underscores.

	s/\'\'/\"/g;
	s/\`\`/\"/g;

	if ((!(/^[\'\"\,\.]/)) && (leading_quote == 0))
	{ 
		print " ";
	}
	print $_;
	if (/\./) { print "\n"; }

	$leading_quote = 0;
	if (/^\"/) { $leading_quote = 1; }

}

