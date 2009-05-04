#! /usr/bin/env perl
#
# dj-clean.pl
#
# Clean out cruft from the Disjuncts, DisjunctSenses tables.
#
# Go through the Disjunct and DisjunctSenses tables, line by line, and
# copy them to a new set of tables, delete entries which contain punctuation,
# web addresses, mis-spelled words.
# And more: delete proper names, delete numbers.
#
# Copyright (c) 2009 Linas Veptas
#
use utf8;
binmode STDIN, ':encoding(UTF-8)';
binmode STDOUT, ':encoding(UTF-8)';
use Encode;

use strict;
use warnings;
use DBI;

# Declare the connection to the database.
my $dbh = DBI->connect('DBI:Pg:dbname=lexat', 'linas', 'asdf')
   or die "Couldn't connect to database: " . DBI->errstr;

# Declare the table names
# my $disj_tbl = "Disjuncts";
# my $sens_tbl = "DisjunctSenses";
my $disj_tbl = "djs";
my $sens_tbl = "djsenses";

my $punc = '\|\(\)\{\}\[\]<>\^\$\&）〈»«~!\?#+@†%=/~;:"…\*\+,„“”‘\-–';
my $quot = '\'’$';

# $punc = decode_utf8($punc);
# $quot = decode_utf8($quot);

# ---------------------------------------------------------------
# Assorted select statements to pull data out.

my $djsel = $dbh->prepare(
	'SELECT inflected_word FROM ' . $disj_tbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $djdel = $dbh->prepare(
	'DELETE FROM ' . $disj_tbl . ' WHERE inflected_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $selsense = $dbh->prepare(
	'SELECT inflected_word FROM ' . $sens_tbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $delsense = $dbh->prepare(
	'DELETE FROM ' . $sens_tbl . ' WHERE inflected_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------------------------------------------------------
# delete_crud -- delete crud from table
# first argument is an SQL select statement returning words
# second arg is am SQL statement that deletes a word.
sub delete_crud
{
	my ($select, $delete) = @_;

	$select->execute()
		or die "Couldn't execute statement: " . $select->errstr;

	my $totcnt = $select->rows;
	print "Will examined $totcnt entries\n";

	# Loop over all rows, and delete rows containing crud.
	my $delcnt = 0;
	my $prdelcnt = 0;
	my $nrdelcnt = 0;
	my $punccnt = 0;
	for (my $i=0; $i<$select->rows; $i++)
	{
		if ($i%10000 == 1)
		{
			my $dt = $delcnt + $prdelcnt + $nrdelcnt;
			my $pt = 100 * $dt / $i;
			print "$i punc=$punccnt delpunc=$delcnt prop=$prdelcnt nr=$nrdelcnt   deltot=$dt  percent=$pt\n";
		}
		my ($word) = $select->fetchrow_array();

		my $orig_word = $word;
		$word = decode_utf8($word);

		# Delete entries that start with capital letters
		if ($word =~ /^[A-Z]/)
		{
			$prdelcnt ++;
			# print "deleting entry $word\n";
			$delete->execute($orig_word);
			next;
		}

		if ($word =~ /[0-9]/)
		{
			$nrdelcnt ++;
			# print "deleting entry $word\n";
			$delete->execute($orig_word);
			next;
		}

		if ($word =~ /[$punc$quot]/)
		#if ($word =~ /\W/)
		{
			$punccnt ++;
 # print "yooo $word\n";
			# single punctuation is OK.
			if ($word =~ /^[$punc]{1}$/) { next; }
			if ($word =~ /^[$quot]{1}$/) { next; }

			# Pure alpha with hyphen or apostrophe is OK
			if ($word =~ /^[\'’\w][\w\-\.\/\&–\'’]+$/) { next; }

			# Delete it!
			# $word = decode_utf8($word);
			# print "deleting entry $word\n";
			$delete->execute($orig_word);
			$delcnt ++;
		}
	}

	$totcnt = $select->rows;
	print "Examined $totcnt entries; $punccnt had punctuation\n";
	print " Deleted $delcnt punctuated entries\n";
	print " Deleted $prdelcnt proper names\n";
	print " Deleted $nrdelcnt numeric entries\n";

	$delcnt += $prdelcnt + $nrdelcnt;
	# return the deleted line count
	$delcnt;
}

# ---------------------------------------------------------------
my $delcnt = 0;

$delcnt = delete_crud($djsel, $djdel);
print "Deleted $delcnt entries total from the $disj_tbl table\n";

# Don't do the djsenses table -- if it has WordNet entries, then
# assume that they are good.
# print "\n\n";
# $delcnt = delete_crud($selsense, $delsense);
# print "Deleted $delcnt entries total from $sens_tbl table\n";
