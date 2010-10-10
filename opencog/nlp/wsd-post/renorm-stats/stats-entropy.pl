#! /usr/bin/env perl
#
# stats-entropy.pl
#
# Display bin-count of entropy for sense assignments.
# The output of this script is just a multi-column, tab-separated
# table of numbers, suitable for graphing, e.g. with gnuplot.
# 
# Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
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
# my $dj_tablename = "Disjuncts";
my $dj_tablename = "NewDisjuncts";

#--------------------------------------------------------------------

sub show_ent
{
	my $nbins = 300;
	my $emax = 4.0;

	my @bins = (0);
	my $i=0;
	for ($i=0; $i<$nbins; $i++) { $bins[$i] = 0; }
	
	# We are going to reject any sense observations for which the 
	# count seemse to be too small. Basically, if there were N different
	# senses that were observed with a word-disjunct pair, then we want 
	# to have seen at least 4*N observations. This discards a *huge*
	# number of observations.
	my $select = $dbh->prepare('SELECT entropy FROM ' . $dj_tablename . 
		' WHERE entropy >= 0.0 AND sense_obscnt > 4 * senses_observed;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $nr = $select->rows;
	print "#\n# bincount of entropy \n";
	print "#\n# Will look at $nr rows in $dj_tablename\n";
	print "#\n# bin into $nbins bins\n#\n";
	for (my $j=0; $j<$select->rows; $j++)
	{
		my ($ent) = $select->fetchrow_array();

		$i = $nbins * $ent / $emax;
		if ($nbins <= $i) { $i = $nbins -1; }

		$bins[$i] ++;
	}

	for ($i=0; $i<$nbins; $i++)
	{
		my $ent = $emax * $i / $nbins;
		my $cnt = $bins[$i];
		print "$i\t$ent\t$cnt\n";
	}
}


show_ent();
