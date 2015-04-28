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
# The 20080605 dump contains 254K sentences.
#
# Usage: "bzless conceptnet_en_20080605.n3.bz2 | ./conceptnet.pl"
#
# Linas Vepstas March 2009
#

use strict;
use warnings;


while (<>)
{
	# Find the sentences
	if (/conceptnet:Sentence "(.+)"\./)
	{
		my $sent = $1;

		my $more = 1;
		do {
			# Trim leading blanks
			$sent =~ s/^\s//;
			$sent =~ s/^\s//;
			$sent =~ s/^\s//;
	
			# Trim trailing blanks
			$sent =~ s/\s$//;
			$sent =~ s/\s$//;
			$sent =~ s/\s$//;

			# Place multiple sentences on individual lines.
			# Err, no. Looks like most multi-sentence constructions are ugly,
			# so don't print them at all.
			if ($sent =~ /(.+?\.)(.+)/)
			{
				# print "$1\n";
				# $sent = $2;
				$more = 0;
			}
			else
			{
				print "$sent\n";
				$more = 0;
			}
		} while ($more);
	}
}
