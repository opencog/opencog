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

my $punc = '\|\(\)\{\}\[\]<>\^\$\&）〈»«~!\?#+@†%=/~;:"…\*\+\.,„“”‘\-–';
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

	# Loop over all rows, and delete rows containing crud.
	my $delcnt = 0;
	my $totcnt = 0;
	for (my $i=0; $i<$select->rows; $i++)
	{
		my ($word) = $select->fetchrow_array();

		my $orig_word = $word;
		$word = decode_utf8($word);
		if ($word =~ /[$punc$quot]/)
		#if ($word =~ /\W/)
		{
 print "yooo $word\n";
			# single punctuation is OK.
			if ($word =~ /^[$punc]{1}$/) { next; }
			if ($word =~ /^[$quot]{1}$/) { next; }

			# Pure-numeric quantities are OK.
			# if ($word =~ /^(AUD|AUD\$|USD|US\$|HK\$|~|=|\.)*[0-9+\-~][0-9+\-\/$punc]*(st|nd|rd|th|am|pm|ft|°C|°F|°)*$/) { next; }

			# Latitude, longitude
			# if ($word =~ /^\d[\d°']+(E|W|N|S)?$/) { next; }

			# dates
			# if ($word =~ /^'\d+$/) { next; }

			# Pure alpha with hyphen or apostrophe is OK
			if ($word =~ /^[\'’\w][\w\-\/\&–\'’]+$/) { next; }

			# Ending with a dot is OK.
			if ($word =~ /^\w\w\-–]+\.$/) { next; }

			# Two, three, four-letter abbreviations
			if ($word =~ /^\w\.[\w\-–]*$/) { next; }
			if ($word =~ /^\w\.\w\.[\w\-–]*$/) { next; }
			if ($word =~ /^\w\.\w\.\w\.[\w\-–]*$/) { next; }
			if ($word =~ /^\w\.\w\.\w\.\w\.[\w\-–]*$/) { next; }
			if ($word =~ /^A.F.L-C.I.O$/) { next; }
			if ($word =~ /^A.F.L.-C.I.O$/) { next; }

			# Delete it!
			# $word = decode_utf8($word);
			print "deleting entry $word\n";
			# $delete->execute($orig_word);
			$delcnt ++;
		}
		$totcnt ++;
	}

	my $punccnt = $select->rows - $totcnt;
	$totcnt = $select->rows;
	print "Examined $totcnt entries; $punccnt had punctuation\n";

	# return the deleted line count
	$delcnt;
}

# ---------------------------------------------------------------
my $delcnt = 0;

$delcnt = delete_crud($djsel, $djdel);
print "Deleted $delcnt corrupted entries from the $disj_tbl table\n";

# $delcnt = delete_crud($selsense, $delsense);
# print "Deleted $delcnt corrupted entries from $sens_tbl table\n";
#
