
#
# sysnset-renorm.pl
#

use WordNet::QueryData;
use WordNet::SenseKey;
use strict;

my $lempos = "run#v#2";

my $wn_dictionary_location = "/usr/share/wordnet";
my $wn = WordNet::QueryData->new($wn_dictionary_location);

my $off = $wn->offset($lempos);

my $sk = WordNet::SenseKey->new($wn_dictionary_location);
my $skey = $sk->get_sense_key($off, $lempos);

print "found the sense $skey for $lempos\n";

