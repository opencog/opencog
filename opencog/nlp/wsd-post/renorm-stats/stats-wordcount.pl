#! /usr/bin/env perl
#
# stats-wordcount.pl
#
# Display bincount (freequency) of occurance of unique
# words in the "disjuncts" table.  This should of course,
# obey Zipf's law.
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

my $nbins = 500;

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
	print "#\n# Will look at $nr words in $dj_tablename\n";
	print "#\n# group into $nbins bins\n#\n";

	my $sum_cnt = 0.0;
	my $sum_obscnt = 0.0;
	for (my $j=0; $j<$select->rows; $j++)
	{
		my ($cnt, $obscnt) = $select->fetchrow_array();
		$sum_cnt += $cnt;
		$sum_obscnt += $obscnt;

print "duuudee $cnt $obscnt\n";
	}
}

show_word_counts();

