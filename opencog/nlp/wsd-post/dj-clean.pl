#! /usr/bin/env perl
#
# dj-clean.pl
#
# Clean out cruft from the pairs table.
#
# Go through the left and right marginal tables, and the pair table, 
# line by line, and delete entries which contain punctuation, 
# web addresses, mis-spelled words. Update the count after doing so.
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
my $unigram = "Unigram";
my $leftmarg = "LeftMarginal";
my $rightmarg = "RightMarginal";
my $pairstbl = "Pairs";
my $linkstbl = "Linkages";
my $lemmas = "Lemmas";
my $lemmaleftmarg = "LeftLemmas";
my $lemmarightmarg = "RightLemmas";
my $lemmastbl = "LemmaPairs";

my $punc = '\|\(\)\{\}\[\]<>\^\$\&）〈»«~!\?#+@†%=/~;:"…\*\+\.,„“”‘\-–';
my $quot = '\'’$';

# $punc = decode_utf8($punc);
# $quot = decode_utf8($quot);

# ---------------------------------------------------------------
# Assorted select statements to pull data out.

my $unisel = $dbh->prepare(
	'SELECT word FROM ' . $unigram)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $unidel = $dbh->prepare(
	'DELETE FROM ' . $unigram . ' WHERE word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $leftsel = $dbh->prepare(
	'SELECT left_word FROM ' . $leftmarg)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $leftdel = $dbh->prepare(
	'DELETE FROM ' . $leftmarg . ' WHERE left_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $rightsel = $dbh->prepare(
	'SELECT right_word FROM ' . $rightmarg)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $rightdel = $dbh->prepare(
	'DELETE FROM ' . $rightmarg . ' WHERE right_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $selpairl = $dbh->prepare(
	'SELECT left_word FROM ' . $pairstbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $delpairl = $dbh->prepare(
	'DELETE FROM ' . $pairstbl . ' WHERE left_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $selpairr = $dbh->prepare(
	'SELECT right_word FROM ' . $pairstbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $delpairr = $dbh->prepare(
	'DELETE FROM ' . $pairstbl . ' WHERE right_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $sellinkl = $dbh->prepare(
	'SELECT left_word FROM ' . $linkstbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $dellinkl = $dbh->prepare(
	'DELETE FROM ' . $linkstbl . ' WHERE left_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $sellinkr = $dbh->prepare(
	'SELECT right_word FROM ' . $linkstbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $dellinkr = $dbh->prepare(
	'DELETE FROM ' . $linkstbl . ' WHERE right_word = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $lemmasel = $dbh->prepare(
	'SELECT lemma FROM ' . $lemmas)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $lemmadel = $dbh->prepare(
	'DELETE FROM ' . $lemmas . ' WHERE lemma = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $lemmaleftsel = $dbh->prepare(
	'SELECT left_lemma FROM ' . $lemmaleftmarg)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $lemmaleftdel = $dbh->prepare(
	'DELETE FROM ' . $lemmaleftmarg . ' WHERE left_lemma = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $lemmarightsel = $dbh->prepare(
	'SELECT right_lemma FROM ' . $lemmarightmarg)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $lemmarightdel = $dbh->prepare(
	'DELETE FROM ' . $lemmarightmarg . ' WHERE right_lemma = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

# ---------------

my $sellemmal = $dbh->prepare(
	'SELECT left_lemma FROM ' . $lemmastbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $dellemmal = $dbh->prepare(
	'DELETE FROM ' . $lemmastbl . ' WHERE left_lemma = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $sellemmar = $dbh->prepare(
	'SELECT right_lemma FROM ' . $lemmastbl)
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $dellemmar = $dbh->prepare(
	'DELETE FROM ' . $lemmastbl . ' WHERE right_lemma = ?')
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
# print "yooo $word\n";
			# single punctuation is OK.
			if ($word =~ /^[$punc]{1}$/) { next; }
			if ($word =~ /^[$quot]{1}$/) { next; }

			# Pure-numeric quantities are OK.
			if ($word =~ /^(AUD|AUD\$|USD|US\$|HK\$|~|=|\.)*[0-9+\-~][0-9+\-\/$punc]*(st|nd|rd|th|am|pm|ft|°C|°F|°)*$/) { next; }
	
			# Latitude, longitude
			if ($word =~ /^\d[\d°']+(E|W|N|S)?$/) { next; }

			# dates
			if ($word =~ /^'\d+$/) { next; }

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

# $delcnt = delete_crud($unisel, $unidel);
# print "Deleted $delcnt corrupted entries from the $unigram table\n";
# 
# $delcnt = delete_crud($leftsel, $leftdel);
# print "Deleted $delcnt corrupted entries from the $leftmarg table\n";
# 
# $delcnt = delete_crud($rightsel, $rightdel);
# print "Deleted $delcnt corrupted entries from the $rightmarg table\n";
# 
# $delcnt = delete_crud($selpairl, $delpairl);
# print "Deleted $delcnt corrupted entries from left side of $pairstbl table\n";
# 
# $delcnt = delete_crud($selpairr, $delpairr);
# print "Deleted $delcnt corrupted entries from right side of $pairstbl table\n";
#
#$delcnt = delete_crud($sellinkl, $dellinkl);
#print "Deleted $delcnt corrupted entries from left side of $linkstbl table\n";
#
#$delcnt = delete_crud($sellinkr, $dellinkr);
#print "Deleted $delcnt corrupted entries from right side of $linkstbl table\n";

# $delcnt = delete_crud($lemmasel, $lemmadel);
# print "Deleted $delcnt corrupted entries from right side of $lemmas table\n";
#
# $delcnt = delete_crud($lemmaleftsel, $lemmaleftdel);
# print "Deleted $delcnt corrupted entries from the $lemmaleftmarg table\n";
# 
# $delcnt = delete_crud($lemmarightsel, $lemmarightdel);
# print "Deleted $delcnt corrupted entries from the $lemmarightmarg table\n";

$delcnt = delete_crud($sellemmal, $dellemmal);
print "Deleted $delcnt corrupted entries from left side of $lemmastbl table\n";

$delcnt = delete_crud($sellemmar, $dellemmar);
print "Deleted $delcnt corrupted entries from right side of $lemmastbl table\n";
