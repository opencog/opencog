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
$sum_parse_score = 0;

$parse_score = 0;
$in_features = 0;
$wrd_count = 0;
$weighted_wrd_count = 0;

while (<>)
{
	if (/<sentence index=/) { $num_sentences ++; next; }
	if (/<parse id=/) { $num_parses ++;  next; }


	# <lg-rank num_skipped_words="0" disjunct_cost="3" and_cost="7" link_cost="47" />

	# Look for the ranking section. It should look like this:
	# <lg-rank num_skipped_words="0" disjunct_cost="2" and_cost="0" link_cost="14" />
	# decode it and build a parse scrore from it.
	if (/<lg-rank/)
	{
		my $num_skipped_words = 0;
		my $disjunct_cost = 0;
		my $and_cost = 0;
		my $link_cost = 0;

		/num_skipped_words=\"(\d+)\"/ and $num_skipped_words = $1;
		/disjunct_cost=\"(\d+)\"/ and $disjunct_cost = $1;
		/and_cost=\"(\d+)\"/ and $and_cost = $1;
		/link_cost=\"(\d+)\"/ and $link_cost = $1;

		# An "arbitrary" parse ranking ... note that this matches
		# the java code in relex ParsedSentence.java
		$parse_score = 0.4 * $num_skipped_words;
		$parse_score += 0.2 * $disjunct_cost;
		$parse_score += 0.06 * $and_cost;
		$parse_score += 0.012 * $link_cost;
		$parse_score = exp(-$parse_score);

		$sum_parse_score += $parse_score;
		next;
	}
	if (/<features>/) { $in_features = 1;  next; }
	if (/<\/features>/) { $in_features = 0;  next; }

	if ($in_features)
	{
		$wrd_count ++;
		$weighted_wrd_count += $parse_score;
		next;
	}
}

print "counted $num_sentences sentences and $num_parses parses\n";

$avg = $sum_parse_score / $num_parses;
print "total of $sum_parse_score for the parse score, or avg=$avg per parse\n";

$avg = $weighted_wrd_count/$wrd_count;
print "wrd-count=$wrd_count and weighted =$weighted_wrd_count or avg=$avg per word\n";
