#! /usr/bin/env perl
#
# stats-wordcount.pl
#
# Display frequency of occurance of words, as a function of rank,
# in the "disjuncts" table.  This should, of course, obey Zipf's law.
# The output of this script is just a multi-column, tab-separated
# table of numbers, suitable for graphing, e.g. with gnuplot.
#
# Actually, this can be used for the word-counts (InflectMarginal table)
# and word-disjunct pairs (Disjuncts table)
# and word-disjunct-sense triples (DisjunctSenses table)
#
# Copyright (C) 2010 Linas Vepstas <linasvepstas@gmail.com>
#

#--------------------------------------------------------------------
# Need to specify the binmodes, in order for \w to match utf8 chars
use utf8;
binmode STDIN, ':encoding(UTF-8)';
binmode STDOUT, ':encoding(UTF-8)';

use DBI;
use strict;
use warnings;

my $dbh = DBI->connect('DBI:Pg:dbname=lexat', 'linas', 'asdf')
	or die "Couldn't connect to database: " . DBI->errstr;

# Table on which to perform data analysis.
# my $dj_tablename = "InflectMarginal";
# my $dj_tablename = "NewInflectMarginal";
# my $dj_tablename = "NewDisjuncts";
my $dj_tablename = "NewDisjunctSenses";

#--------------------------------------------------------------------

sub show_word_counts
{
	# select * from newinflectmarginal order by count DESC;
	# Actually, look only at those with a parse-confidence
	# greater than zero, as the rest is just borken crud ... 
	my $select = $dbh->prepare('SELECT count, obscnt FROM '.
		$dj_tablename . ' WHERE count > 0 ORDER BY count DESC;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $nr = $select->rows;
	print "#\n# bincount for word frequency \n";
	print "# expect this to obey Zipf's law ... \n";
	print "#\n# Will look at $nr words in $dj_tablename\n";
	print "#\n#\n";

	my $sum_count = 0.0;
	my $sum_obscnt = 0.0;

	# The counts from NewInflectMarginal tables.
	# my $norm_cnt = 3821443.36619242;
	# my $norm_obscnt = 27093506;

	my $binsz = 1;
	for (my $j=0; $j<$select->rows; $j++)
	{
		my ($cnt, $obscnt) = $select->fetchrow_array();

		$sum_count += $cnt;
		$sum_obscnt += $obscnt;

		if ((($j % $binsz) == 0))
		{
			my $k = $j + 1;
			# $cnt /= $norm_cnt;
			# $obscnt /= $norm_obscnt;
			print "$k	$cnt	$obscnt\n";

			# adjust the binsize so we don't print every row ... 
			$binsz = log($k) - 4;
			$binsz = int($binsz);
			$binsz = exp($binsz);
			$binsz = int($binsz);
			if ($binsz < 1) { $binsz = 1; }
		}
	}

	print "# sum_count = $sum_count   sum_obscnt=$sum_obscnt\n";
}

show_word_counts();

