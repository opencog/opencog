#! /usr/bin/env perl

# XML::Simple complains about the xml markup used in the semcore files :-(
# Grrr so does XML::SimpleObject
#
# use XML::Simple;
# my $xs1 = XML::Simple->new();
# my $doc = $xs1->XMLin("-");

use XML::SimpleObject;

$xml = "";

# Loop -- read from stdin, and put quotes around the attributes.
while(<>)
{
	s/=/=\"/g;
	split;
	foreach(@_)
	{
		if (/=/) { 
			if (/>/) {
				s/>/\">/;
				$xml .= $_;
			} else {
				$xml .= $_;
				$xml .= "\"";
			}
		} else {
			$xml .= $_;
		}
		$xml .= " ";
	}
	$xml .= "\n";
}
print $xml;

# my $xmlobj = new XML::SimpleObject(XML => $xml);



