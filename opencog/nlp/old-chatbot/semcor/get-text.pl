#! /usr/bin/env perl
#
# This file just stripes out the sentences from the semcor files.
# Unfortunately, the XML::SimpleParser is buggy as all get-out, 
# and is unusable. Booooooo!
#
# XML::Simple complains about unquoted attribute values in semcor files.
# That's just stupid.
# Grrr so does XML::SimpleObject
# WTF Don't actual human beings actually use this shit?
#
# use XML::Simple;
# my $xs1 = XML::Simple->new();
# my $doc = $xs1->XMLin("-");

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
# print $xml;

# Grrrr XML::SimpleObject does not preserve the order of the children! WTF!
# That is the most inane, insulting bug to date.  Arghhh!
#
use XML::SimpleObject;
my $xmlobj = new XML::SimpleObject(XML => $xml);
foreach $para ($xmlobj->child("contextfile")
                        ->child("context")
                        ->child("p"))
{
	foreach $sent ($para->child("s"))
	{
		@ch = $sent->children;
		foreach $elt (@ch)
		{
			$name = $elt->name;
			if ($name =~ "wf")
			{
				$word = $elt->value;
				$word =~ s/_/ /g;
				if (!($word =~ /^\'/)) { 
					print " ";
				}
				print $word;
			}
			if ($name =~ "punc")
			{
				print $elt->value;
			}
		}
		print "\n";
	}
}


