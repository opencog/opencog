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
	my %wdj_freq = ();
	my %wdj_entropy = ();
	my %wdj_count = ();

	my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0 ORDER BY count DESC;' )
		or die "Couldn't prepare statement: " . $dbh->errstr;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	# First, total up the marginal counts
	my $tot_count = 0.0;
	my $items = 0;
	for (my $i=0; $i<$select->rows; $i++)
	{
		my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();
		$items ++;
		$tot_count += $count;

		my $pair = $infword . "%%%" . $disjunct;
		$wdj_freq{$pair} += $count;
		$wdj_count{$pair} ++;
	}
	print "Total count is $tot_count for $items items\n";

	# Next, compute and store the marginal probabilites
	my $update = $dbh->prepare(
		'UPDATE DisjunctSenses SET log_cond_probability = ? WHERE word_sense = ? AND inflected_word = ? AND disjunct = ?')
		or die "Couldn't prepare statement: " . $dbh->errstr;

	my $olog_2 = -1.0/log(2.0);
	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;
	
	for (my $i=0; $i<$select->rows; $i++)
	{
		my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();
		my $pair = $infword . "%%%" . $disjunct;
		my $tot = $wdj_freq{$pair};
		my $prob = $count / $tot;
		my $ln = log ($prob) * $olog_2;

		$wdj_entropy{$pair} += $prob * $ln;

		$update->execute($ln, $sense, $infword, $disjunct)
			or die "Couldn't execute statement: " . $update->errstr;
	}

	print "Done updating the probs\n";

	# Now update the entropies and the sense-counts.
	my $ups = $dbh->prepare(
		'UPDATE Disjuncts SET entropy = ?, senses_observed = ? WHERE inflected_word = ? AND disjunct = ?');
	
	my $djcnt = 0;
	my $sncnt = 0;
	my $encnt = 0;
	foreach my $pair (keys %wdj_entropy)
	{
		my ($infword, $disjunct) = split(/%%%/, $pair);
		my $entropy = $wdj_entropy{$pair};
		my $sense_count = $wdj_count{$pair};

		$djcnt ++;
		$sncnt += $sense_count;
		$encnt += $entropy;
		$ups->execute($entropy, $sense_count, $infword, $disjunct)
			or die "Couldn't execute statement: " . $update->errstr;
	}

	$sncnt /= $djcnt;
	$encnt /= $djcnt;
	print "Done updating the entropy of $djcnt disjuncts\n"
	print "    Avg sense cnt=$sncnt  avg entropy=$encnt;\n";
}


compute_prob();
