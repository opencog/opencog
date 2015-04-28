#! /usr/bin/env perl
#
# dump-djs.pl
#
# Dump disjunct senses table to comma-seprated-values file.
# This ia s quick hack, so as to import the dataset into data
# analysis programs.
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

my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0 ORDER BY count DESC;' )
	or die "Couldn't prepare statement: " . $dbh->errstr;

$select->execute()
	or die "Couldn't execute statement: " . $select->errstr;

print "sense\tword\tdisjunct\tcount\n";
for (my $i=0; $i<$select->rows; $i++)
{
	my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();
	print "$sense\t$infword\t$disjunct\t$count\n";
}


