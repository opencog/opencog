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

my $sense_key = "run_away%2:38:00::";
my $sense_str = $sk->get_sense($sense_key);

my @synset = $wn->querySense($sense_str, "syns");
foreach (@synset)
{
	my $lempos = $_;
	my $off = $wn->offset($lempos);
	my $skey = $sk->get_sense_key($off, $lempos);
	print "found the sense $skey for $lempos\n";
}



