#! /usr/bin/env perl
#
# stats-entropy.pl
#
# Display bin-count of entropy for sense assignments.
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

#--------------------------------------------------------------------

sub show_ent
{
	my $nbins = 300;
	my $emax = 4.0;

	my @bins = (0);
	my $i=0;
	for ($i=0; $i<$nbins; $i++) { $bins[$i] = 0; }
	
	my $select = $dbh->prepare('SELECT entropy FROM Disjuncts WHERE entropy >= 0.0;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $nr = $select->rows;
	print "#\n# bincount of entropy \n";
	print "#\n# Will look at $nr rows in Disjuncts\n";
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
