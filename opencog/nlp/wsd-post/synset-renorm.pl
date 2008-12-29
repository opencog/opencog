#! /usr/bin/env perl
#
# synset-renorm.pl
#
# Rework the database contents so as to combine multiple synset entries
# into one. This step is needed in part because the original WSD code
# handled synsets in a buggy way ... which still needs to be fixed.

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
my $sk = WordNet::SenseKey->new($wn_dictionary_location);

# get_synset -- return a wordnet synset.
# Given a sense key as input, this will
# return a list of sense keys in the synset.
sub get_synset
{
	my ($sense_key) = @_;
	my $sense_str = $sk->get_sense($sense_key);

	my @synset = $wn->querySense($sense_str, "syns");
	my @keyset = ();
	foreach (@synset)
	{
		my $lempos = $_;
		my $off = $wn->offset($lempos);
		my $skey = $sk->get_sense_key($off, $lempos);
		push @keyset, $skey;
	}

	return @keyset;
}

# ----------------------------------------------------------

# get_lemma_sense -- get matching lemma from a synset.
# Return an alternate sense key that belongs to the same
# synset ass the input sense key, but has the the lemmatized
# form $lemma at its root.
#
# Thus, for example:
#
#      get_lemma_sense("join#v", "connect%2:42:02::");
#
# will return "join%2:42:01", because "join%2:42:01" is in the same
# synset as "connect%2:42:02::", but has "join" as its root. 
#
sub get_lemma_sense
{
	my ($lemma, $sense) = @_;

	# strip off the part-of-speech marker from the lemma.
	$lemma =~ m/([\w\.]+)#/;
	if (defined($1))
	{
		$lemma = $1;
	}

	# Loop over the synset, looking for a matching form.
	my @synset = get_synset($sense);
	foreach (@synset)
	{
		my $altsense = $_;
		$altsense =~ m/([\w\.]+)%/;
		if ($1 eq $lemma)
		{
			return $altsense;
		}
	}

	my $notfound; # this is undefined!
	return $notfound;
}

# ----------------------------------------------------------

sub update_record
{
	my ($sense, $infword, $disjunct, $count) = @_;

	print "duuude update $sense, $infword, $disjunct, $count\n";
}

# ----------------------------------------------------------
my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0;')
	or die "Couldn't prepare statement: " . $dbh->errstr;

$select->execute()
	or die "Couldn't execute statement: " . $select->errstr;

for (my $i=0; $i<$select->rows; $i++)
{
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
	my $canon_sense = get_lemma_sense($lemma, $sense);
	if (!defined($canon_sense))
	{
		next;
	}

	# That's it -- we've got a canonical sense. Now update the 
	# database.
	update_record ($canon_sense, $infword, $disjunct, $count);
}

