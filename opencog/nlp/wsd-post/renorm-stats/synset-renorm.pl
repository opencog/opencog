#! /usr/bin/env perl
#
# synset-renorm.pl
#
# Rework the database contents so as to combine multiple synset entries
# into one. This step is needed in part because the original WSD code
# failed to realize that multiple senses might belong to the same
# synset.  ... which still needs to be fixed.
#
# Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
#

use utf8;
binmode STDIN, ':encoding(UTF-8)';
binmode STDOUT, ':encoding(UTF-8)';

use strict;
use warnings;

use WordNet::QueryData;
use WordNet::SenseKey;
use DBI;

my $dbh = DBI->connect('DBI:Pg:dbname=lexat', 'linas', 'asdf')
   or die "Couldn't connect to database: " . DBI->errstr;

# ----------------------------------------------------------
my $wn_dictionary_location = "/usr/share/wordnet";
my $wn = WordNet::QueryData->new($wn_dictionary_location);
my $sk = WordNet::SenseKey->new($wn);

# ----------------------------------------------------------
my $selectup = $dbh->prepare(
	'SELECT (count) FROM DisjunctSenses WHERE ' . 
	'inflected_word = ? AND disjunct = ? AND word_sense = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $insertup = $dbh->prepare(
	'INSERT INTO DisjunctSenses ' .
	'(inflected_word, disjunct, word_sense, count) VALUES (?,?,?,?)')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $updateup = $dbh->prepare(
	'UPDATE DisjunctSenses SET count = ? WHERE ' .
	'word_sense = ? AND inflected_word = ? AND disjunct = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $inserted = 0;
my $updated = 0;

sub update_record
{
	my ($sense, $infword, $disjunct, $count) = @_;

	# print "duuude update $sense, $infword, $disjunct, $count\n";
	$selectup->execute($infword, $disjunct, $sense)
		 or die "Couldn't execute statement: " . $selectup->errstr;

	if ($selectup->rows == 0)
	{
		$insertup->execute($infword, $disjunct, $sense, $count);
		$inserted ++;
		return;
	}

	# update the current count
	my ($curr_cnt) = $selectup->fetchrow_array();
	$count += $curr_cnt;

	$updateup->execute($count, $sense, $infword, $disjunct);
	$updated ++;
}

# ----------------------------------------------------------

my $delete = $dbh->prepare(
	'DELETE FROM DisjunctSenses WHERE ' . 
	'word_sense = ? AND inflected_word = ? AND disjunct = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0;')
	or die "Couldn't prepare statement: " . $dbh->errstr;

print "Starting to renormalize synsets in the DisjunctSenses table\n";

$select->execute()
	or die "Couldn't execute statement: " . $select->errstr;

print "Will examine $select->rows rows in the DisjunctSenses table\n"

my $examined = 0;
for (my $i=0; $i<$select->rows; $i++)
{
	$examined ++;
	if ($examined %10000 == 0)
	{
		print "So far, examined=$examined updated=$updated inserted=$inserted\n";
	}
	my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();

	# Extract the lemma form from the sense key, and from the word.
	$sense =~ m/([\w\.]+)%(\d+)/;
	my $sense_lemma = $1;
	my $slemma = $sense_lemma;
	my $pos = $2;

	$pos =~ s/1/n/;
	$pos =~ s/2/v/;
	$pos =~ s/3/a/;
	$pos =~ s/4/r/;

	if ($pos =~ /5/)
	{
		print "Don't know what to do! $sense $infword $count\n";
		next;
	}

	my $word = $infword;
	$infword =~ m/(\w+)\.(\w+)/;
	if (defined($1))
	{
		$word = $1;
	}

	# Build a pseudo-word-pos string from the inflected word.
	if (defined ($pos))
	{
		$word = $word . "#" . $pos;
		$slemma = $slemma . "#" . $pos;
	}

	# Get its lemmatized form.
	my @forms = $wn->validForms($word);
	my $lemma = $forms[0];

	# Very strange, but in rare cases, there's no lemma. Examples:
	# cannot#v and for#v -- but how these got into the database to
	# begin with is unclear. Anyway, punt on these.
	if (!defined($lemma))
	{
		# print "wtf no lemma for word=$word (sense=$sense inf=$infword)\n";
		next;
	}

	# Are these equal? If so, then we're good. Else, update the stats.
	if ($slemma eq $lemma)
	{
		next;
	}

	# Find the canonical sense for this word.
	my $canon_sense = $sk->get_canonical_sense($lemma, $sense);
	if (!defined($canon_sense))
	{
		next;
	}

	# That's it -- we've got a canonical sense. Now update the 
	# database with the new count, and delete the old record.
	update_record ($canon_sense, $infword, $disjunct, $count);
	$delete->execute($sense, $infword, $disjunct);
}

print "In total, examined=$examined updated=$updated inserted=$inserted\n";
