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

my $dbh = DBI->connect('DBI:Pg:dbname=lexat', 'linas', 'asdf')
   or die "Couldn't connect to database: " . DBI->errstr;

my $select = $dbh->prepare('SELECT * FROM DisjunctSenses WHERE count > 0.0;')
	or die "Couldn't prepare statement: " . $dbh->errstr;

$select->execute()
	or die "Couldn't execute statement: " . $select->errstr;

for (my $i=0; $i<$select->rows; $i++)
{
	my ($sense, $infword, $disjunct, $count, $lp) = $select->fetchrow_array();

	# Extract the lemma form from the sense key, and from the word.
	$sense =~ m/(\w+)%(\d+)/;
	my $sense_lemma = $1;
	my $pos = $2;
	$infword =~ m/(\w+)\.(\w+)/;
	my $word = $1;
	my $flect = $2;

	my @forms = $wn->validForms($word);
	my $lemma = $forms[0];

print "duude $sense, $infword  $sense_lemma $word $lemma\n";

}


my $sense_key = "run_away%2:38:00::";
my @synset = get_synset($sense_key);

foreach (@synset)
{
	print "its $_\n";
}
