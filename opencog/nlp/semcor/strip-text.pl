#! /usr/bin/env perl
#
# This utility just strips out the sentences from the semcor files.
#

# Loop -- read from stdin, and put quotes around the attributes.
$leading_quote = 0;
while(<>)
{
	chop;
	s/<[\/\:\$\'\(\)\w\s=-]+>//g;  # strip out all xml markup

	s/^ //;
	s/_/ /g;  # replace underscores.

	s/\'\'/\"/g;
	s/\`\`/\"/g;  # replace double-quotes.

	if (!($_)) { next; }  # skip empty lines

	if ((!(/^[\'\"\,\.]/)) && ($leading_quote == 0) && (!(/\'/)))
	{ 
		print " ";
	}

	$leading_quote = 0;
	if (/^\"/) { $leading_quote = 1; }

	print $_;
	if (/\?/) {print " "; }
	if (/\./) { print "\n"; $leading_quote = 1; }


}

