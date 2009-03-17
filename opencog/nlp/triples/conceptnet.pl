#! /usr/bin/env perl
#
# conceptnet.pl
#
# Extract English sentences from an MIT ConceptNet file dump.

# ConceptNet consists of a set of triples, for example:
# IsA(Hockey, a sport), together with English language sentences that
# provide evidence or support for the triple, for example: "Hockey is 
# a sport played in an arena". This script extracts those sentences,
# one per line.
#
# Usage: "bzless conceptnet_en_20080605.n3.bz2 | ./conceptnet.pl"
#
# Linas Vepstas March 2009
#

use strict;
use warnings;


while (<>)
{
	# if (/conceptnet:Sentence "(\w+)"\./)
	if (/conceptnet:Sentence "(.+)"\./)
	{
		print "$1\n";
	}
}
