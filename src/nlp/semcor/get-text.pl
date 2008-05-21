#! /usr/bin/env perl

# XML::Simple complains about the xml markup used in the semcore files :-(
# use XML::Simple;
# my $xs1 = XML::Simple->new();
# my $doc = $xs1->XMLin("-");

use XML::SimpleObject;

$xml = "";

while(<>)
{
	$xml = $xml . $_;
}

my $xmlobj = new XML::SimpleObject(XML => $xml);



