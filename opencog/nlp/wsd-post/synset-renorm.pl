#! /usr/bin/env perl
#
# synset-renorm.pl
#

use WordNet::QueryData;
use WordNet::SenseKey;
use strict;

my $wn_dictionary_location = "/usr/share/wordnet";
my $wn = WordNet::QueryData->new($wn_dictionary_location);
my $sk = WordNet::SenseKey->new($wn_dictionary_location);

# Given a since sense key as input, this will
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


my $sense_key = "run_away%2:38:00::";
my @synset = get_synset($sense_key);

foreach (@synset)
{
	print "its $_\n";
}
