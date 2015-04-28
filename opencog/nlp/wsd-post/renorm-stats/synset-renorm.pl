#! /usr/bin/env perl
#
# synset-renorm.pl
#
# Rework the database contents so as to combine multiple synset entries
# into one. This step is needed in part because the original WSD code
# failed to realize that multiple senses might belong to the same
# synset.  ... which still needs to be fixed.
#
# Copyright (C) 2008, 2009 Linas Vepstas <linasvepstas@gmail.com>
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

# Specify the table-name to be cleaned up.
# my $djs_tablename = "DisjunctSenses";
my $djs_tablename = "NewDisjunctSenses";

# ----------------------------------------------------------
my $wn_dictionary_location = "/usr/share/wordnet";
my $wn = WordNet::QueryData->new($wn_dictionary_location);
my $sk = WordNet::SenseKey->new($wn);

# ----------------------------------------------------------
my $selectup = $dbh->prepare(
	'SELECT count, obscnt FROM ' . $djs_tablename . ' WHERE ' . 
	'inflected_word = ? AND disjunct = ? AND word_sense = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $insertup = $dbh->prepare(
	'INSERT INTO ' . $djs_tablename . ' ' .
	'(inflected_word, disjunct, word_sense, count, obscnt) VALUES (?,?,?,?,?)')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $updateup = $dbh->prepare(
	'UPDATE ' . $djs_tablename . ' SET count = ? , obscnt = ? WHERE ' .
	'word_sense = ? AND inflected_word = ? AND disjunct = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $inserted = 0;
my $updated = 0;


# update_record -- update just one record ...
#
sub update_record
{
	my ($sense, $infword, $disjunct, $count, $obscnt) = @_;

	# print "duuude update $sense, $infword, $disjunct, $count, $obscnt\n";
	$selectup->execute($infword, $disjunct, $sense)
		 or die "Couldn't execute statement: " . $selectup->errstr;

	if ($selectup->rows == 0)
	{
		$insertup->execute($infword, $disjunct, $sense, $count, $obscnt);
		$inserted ++;
		return;
	}

	# update the current count
	my @arr = $selectup->fetchrow_array();
	# print "duuude arr @arr\n";
	my ($curr_cnt, $curr_obs) = @arr;
	$count += $curr_cnt;
	$obscnt += $curr_obs;

	$updateup->execute($count, $obscnt, $sense, $infword, $disjunct);
	$updated ++;
}

# ----------------------------------------------------------

my $delete = $dbh->prepare(
	'DELETE FROM ' . $djs_tablename . ' WHERE ' . 
	'word_sense = ? AND inflected_word = ? AND disjunct = ?')
	or die "Couldn't prepare statement: " . $dbh->errstr;

my $select = $dbh->prepare('SELECT * FROM ' . $djs_tablename . 
	' WHERE count > 0.0;')
	or die "Couldn't prepare statement: " . $dbh->errstr;

print "Starting to renormalize synsets in the $djs_tablename table\n";

$select->execute()
	or die "Couldn't execute statement: " . $select->errstr;

my $nrows = $select->rows;
print "Will examine $nrows rows in the $djs_tablename table\n";

my $examined = 0;
for (my $i=0; $i<$select->rows; $i++)
{
	$examined ++;
	if ($examined %10000 == 0)
	{
		print "So far, examined=$examined updated=$updated inserted=$inserted\n";
	}
	my ($sense, $infword, $disjunct, $count, $obscnt, $lp) = $select->fetchrow_array();

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
		print "Warning: Don't know what to do! $sense $infword $count $obscnt\n";
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

	# Some texts have a c# in them, and this confuses wordnet
	# So, if the word has a # in it, we pass ...
	# Actually, the problem is C# returns every word starting
	# with the letter C, leading to an overflow.
	if ($infword =~ /#/)
	{
		# print "Warning: skipping $infword because it has a sharp in it\n";
		# print "\tLine $examined sense $sense\n";
		next;
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
	update_record ($canon_sense, $infword, $disjunct, $count, $obscnt);
	$delete->execute($sense, $infword, $disjunct);
}

print "In total, examined=$examined updated=$updated inserted=$inserted\n";
