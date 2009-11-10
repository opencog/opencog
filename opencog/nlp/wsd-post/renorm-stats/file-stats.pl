#! /usr/bin/env perl
#
# file-stats.pl
#
# Collect some miscellaneous statistics about CFF-format file contents.
#
# Linas Vepstas November 2009
#

$num_sentences = 0;
$num_parses = 0;

while (<>)
{
	if (/<sentence index=/) { $num_sentences ++; }
	if (/<parse id=/) { $num_parses ++; }
}

print "counted $num_sentences sentences and $num_parses parses\n";

