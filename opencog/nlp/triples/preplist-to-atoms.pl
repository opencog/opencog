#! /usr/bin/env perl
#
# preplist-to-atoms.pl
#
# Convert list of prepositions to OpenCog links.
#
# This perl script takes, as input, a list of prepositions, and outputs
# OpenCog links of the form below:
#
# (ListLink (stv 1.0 1.0)
#    (DefinedLinguisticRelationshipNode "behind")
#    (WordNode "behind")
# )
#
# Such ListLinks are needed for the reasoning code in this directory.
#
# To use, do the following:
#
#    cat preplist.txt | ./preplist-to-atoms.pl
#
# Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
#

use strict;

print "scm\n\n";
# Read from standard input, until theres no more standard input.
#
my $pcnt = 0;
while(<>)
{

	# Ignore comments (actually, reproduce them for ease of debugging)
	if(/^;/)
	{
		print "$_";
		next;
	}

	if (/^\s$/)
	{
		next;
	}

	chop;
	print "(ListLink\n";
	print "  (DefinedLinguisticRelationshipNode \"$_\")\n";
   print "  (WordNode \"$_\")\n";
	print ")\n";

	$pcnt ++;
}

print "; Processed $pcnt prepositions\n";
print "\n.\nexit\n";

