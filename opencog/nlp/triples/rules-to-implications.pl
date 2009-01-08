#! /usr/bin/env perl
#
# rules-to-implications.pl
#
# Convert Relex framing rule files to OpenCog ImplicationLinks
#
# This perl script takes, as input, implication rules, in the IF .. THEN
# format used by the RelEx framing code, and generates equivalent
# OpenCog ImplicationLinks.
#

my $in_statement = 0;

while(<>)
{
	# New implications are marked by a hash mark at the begining of the
	# line.
	if(/^#/)
	{
		# close the previous statement
		if ($in_statement)
		{
			printf ")\n";
		}
		$in_statement = 1;
		print "(ImplicationLink\n";
		print "   (AndLink\n";
	}
}

# close the previous statement
if ($in_statement)
{
	printf ")\n";
	$in_statement = 0;
}
