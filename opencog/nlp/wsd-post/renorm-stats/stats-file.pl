#! /usr/bin/env perl
#
# file-stats.pl
#
# Collect some miscellaneous statistics about CFF-format file contents.
# The output of this script is just a multi-column, tab-separated
# table of numbers, suitable for graphing, e.g. with gnuplot.
#
# Usage: cat *.cff | ./file-stats.pl
#
# Runtime might take 10-20 minutes for 50K files.
#
# Copyright (c) 2009, 2010 Linas Vepstas
#

$num_sentences = 0;
$num_parses = 0;
$num_good_parses = 0;
$num_bad_parses = 0;

$sum_parse_score = 0;
$sum_good_parse_score = 0;
$sum_bad_parse_score = 0;

$parse_score = 0;
$in_features = 0;
$wrd_count = 0;
$wrd_good_count = 0;
$weighted_wrd_count = 0;
$weighted_wrd_good_count = 0;

$is_bad = 0;
while (<>)
{
	# count number of sentences, and number of parses
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
		if ($num_skipped_words > 0)
		{
			$num_bad_parses ++;
			$sum_bad_parse_score += $parse_score;
			$is_bad = 1;
		}
		else
		{
			$num_good_parses ++;
			$sum_good_parse_score += $parse_score;
			$is_bad = 0;
		}
		next;
	}
	if (/<features>/) { $in_features = 1;  next; }
	if (/<\/features>/) { $in_features = 0;  next; }

	# Count number of words in a sentence (err, a parse,
	# technically speaking).
	if ($in_features)
	{
		$wrd_count ++;
		$weighted_wrd_count += $parse_score;
		if (0 == $is_bad)
		{
			$wrd_good_count ++;
			$weighted_wrd_good_count += $parse_score;
		}
		next;
	}
}

print "counted $num_sentences sentences and $num_parses parses\n";
$avg = $num_parses / $num_sentences;
print "average of $avg parses per sentence\n";
$avg = $wrd_count / $num_parses;
print "average of $avg words per sentence\n";


print "num good parses: $num_good_parses\n";
print "num bad parses: $num_bad_parses\n";


$avg = $sum_parse_score / $num_parses;
print "total of $sum_parse_score for the parse score, or avg=$avg per parse\n";

$avg = $sum_good_parse_score / $num_good_parses;
print "total of $sum_good_parse_score for the good parse score, or avg=$avg per parse\n";

$avg = $sum_bad_parse_score / $num_bad_parses;
print "total of $sum_bad_parse_score for the bad parse score, or avg=$avg per parse\n";

$avg = $weighted_wrd_count/$wrd_count;
print "wrd-count=$wrd_count and weighted =$weighted_wrd_count or avg=$avg per word\n";

$avg = $weighted_wrd_good_count/$wrd_good_count;
print "wrd-good-count=$wrd_good_count and weighted =$weighted_wrd_good_count or avg=$avg per word\n";

