#! /usr/bin/env perl
#
# dj-probs.pl
#
# Compute various probabilities and marginal probabilites for the
# wordsense-word-disjunct dataset.
# 
# Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

sub compute_prob
{
	my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0 ORDER BY count DESC;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $tot_count = 0.0;
	for (my $i=0; $i<$select->rows; $i++)
	{
		my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();
		$tot_count += $count;
	}

}
