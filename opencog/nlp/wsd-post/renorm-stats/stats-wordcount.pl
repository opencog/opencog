#! /usr/bin/env perl
#
# stats-wordcount.pl
#
# Display frequency of occurance of words, as a function of rank,
# in the "disjuncts" table.  This should, of course, obey Zipf's law.
# The output of this script is just a multi-column, tab-separated
# table of numbers, suitable for graphing, e.g. with gnuplot.
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
my $dj_tablename = "NewInflectMarginal";

#--------------------------------------------------------------------

sub show_word_counts
{
	# select * from newinflectmarginal order by count DESC;
	my $select = $dbh->prepare('SELECT count, obscnt FROM '.
		$dj_tablename . ' ORDER BY count DESC;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $nr = $select->rows;
	print "#\n# bincount for word frequency \n";
	print "# expect this to obey Zipf's law ... \n";
	print "#\n# Will look at $nr words in $dj_tablename\n";
	print "#\n#\n";

	my $binsz = 1;
	for (my $j=0; $j<$select->rows; $j++)
	{
		my ($cnt, $obscnt) = $select->fetchrow_array();

		if ((($j % $binsz) == 0))
		{
			my $k = $j + 1;
			print "$k	$cnt	$obscnt\n";

			# adjust the binsize so we don't print every row ... 
			$binsz = log($k) - 4;
			$binsz = int($binsz);
			$binsz = exp($binsz);
			$binsz = int($binsz);
			if ($binsz < 1) { $binsz = 1; }
		}
	}
}

show_word_counts();

