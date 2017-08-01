#! /usr/bin/env perl
#
#  wiki-scrub.pl
#
# Ad-hoc script to scrub wikipedia xml dumps, outputting only valid
# English-language sentences.  This  script removes wiki markup, URL's
# tables, images, & etc.  It currently seems to be pretty darned
# bullet-proof, although it might handle multi-line refs incorrectly.
# Can also stumble over nested elements...
# 
# Example usage:
# cat simplewiki-20080629.xml.bz2 | bunzip2 | ./wiki-scrub.pl
# 
# This script creates individual files, one per article, placing these
# in a directory "wiki-stripped", which must already exist. This
# directory name is user-configurable, see immediately below.
#
# Copyright (c) 2008, 2013 Linas Vepstas <linas@linas.org>
#
# TODO: Remove }} .. there seem to be a bunch of these left over.
# also img pix stuff seems to stick around ...
#
#--------------------------------------------------------------------
#
# Directory where to dump the stripped pages.
# this directory must already exist

$page_out_directory = "wiki-stripped";

#--------------------------------------------------------------------
# Need to specify the binmodes, in order for \w to match utf8 chars
use utf8;
binmode STDIN, ':encoding(UTF-8)'; 
binmode STDOUT, ':encoding(UTF-8)';

$have_text = 0;
$have_infobox = 0;
$have_table = 0;
$have_ptable = 0;
$have_cmnt = 0;
$notfirst = 0;
$page_title = "";
$page_not_open = 1;
$start_processing = 0;

while (<>)
{
	if (/<title>(.+?)<\/title>/) {
		$page_title = $1;
		close PAGE;
		$page_not_open = 1;
	}
	if (/<text xml:space/) { $have_text = 1; $start_processing = 1; }

	if (!$start_processing) { next; }

	# End of a wiki page.
	# If there are any badly-formed tables, etc. then reset the state
	# variables as we exit from a wiki page.
	if (/<\/text>/) {
		$have_text = 0;
		$have_infobox = 0;
		$have_table = 0;
		$have_ptable = 0;
		$have_cmnt = 0;
		$notfirst = 0;
		$start_processing = 0;
	}

	chop;

	# remove the text xml
	s/.*<text xml:space="preserve">//;

	# kill redirect pages
	if (/#REDIRECT/) { $have_text = 0; next; }
	if (/#redirect/) { $have_text = 0; next; }

	# Remove stuff that's commented out. Don't be greedy(?)!
	# Do this before most other processing.
	s/&lt;!--.+?--&gt;//g;
	if (/&lt;!--/) { $have_text = 0; next; }
	if (/--&gt;/) { $have_text = 1; next; }

	# kill photo galleries
	if (/&lt;gallery&gt;/) { $have_text = 0; }
	if (/&lt;gallery .+?&gt;/) { $have_text = 0; }
	if (/&lt;\/gallery&gt;/) { $have_text = 1; next; }

	# kill tables. These start with {| or |- and end with |}
	# tables may be nested.
	if (/^:*(\{\||\|\-)/) { $have_text = 0; $have_table++; }
	if ($have_table && /^\|\}\s*/) {
		$have_table --;
		if (0 == $have_table) { $have_text = 1; }
		next;
	}
	if ($have_table) { next; }

	if (/&lt;table/) { $have_text = 0; $have_ptable++; }
	if (/&lt;\/table/) {
		$have_ptable --;
		if (0 == $have_ptable) { $have_text = 1; }
		next;
	}
	if ($have_ptable) { next; }

	# kill stuff like this: 172||9||23||2||30||1||225||12
	# or this: 118||2||||||||||||||118||2
	# or this: !Total||105||37
	# Sometimes it doesn't start or end with a number, and also there could be
	# markup in between like: ||11||3||colspan=&quot;2&quot;|-||11||3
	s/(.*\|\|)+\d*$//;

	# Ignore single-line templates e.g. {{template gorp}}
	# Also nested ones e.g. {{math|{{aao|300|120|+}}}}
	# Do this before processing multi-line templates
	while(/\{\{[^{]+?\}\}/) { s/\{\{[^{]+?\}\}// };

	# kill infoxes and other multi-line templates. These may have
	# embedded templates.
	# Don't be greedy -- some of these, like {{cite}}, have valid text
	# both before and after.
	my @cb = /\}\}/g;
	if ($have_infobox && @cb) {
		$have_infobox -= scalar @cb;
		if (0 == $have_infobox) {
			s/.*\}\}//;
		}
	}
	my @ob = /\{\{/g;
	if (@ob) {
		if ($have_infobox) {
			$have_infobox += scalar @ob;
		} else {
			$have_infobox += scalar @ob;
			$notfirst = 0;
			s/\{\{.+$//;
		}
	}
	if ($have_infobox) {
		if ($notfirst) { next; }
		$notfirst = 1;
	}

	# remove single-line math markup. Don't be greedy(?)!
	# Do this before multi-line math markup.
	s/&lt;math&gt;.+?&lt;\/math&gt;//g;

	# kill multi-line math markup
	if (/&lt;math&gt;/) { $have_text = 0; }
	if (/&lt;\/math&gt;/) { $have_text = 1; next; }

	# ignore everything that isn't in a text section.
	if (0 == $have_text) { next; }

	# remove triple and double quotes (wiki bold, italic)
	s/\'\'\'//g;
	s/\'\'//g;

	# remove refs, assumed to sit on one line. Don't be greedy(?)!
	s/&lt;ref.+?&lt;\/ref&gt;//g;

	# multi-line refs seem to only have {{cite}} inside of them.
	# The below seems to work, but should probably be convertedto work 
	# like multi-line templates.
	s/&lt;ref&gt;//g;
	s/&lt;\/ref&gt;//g;
	s/&lt;ref name.+?&gt;//g;

	# Ignore everything of the form ^[[en:title]] (these are tranlsated
	# pages) These sometimes have {{Link FA|en}} after them.
	if (/^\[\[\w[\w-]+?:.+?\]\]( \{\{Link FA\|\w+\}\})*$/) { next; }

	# Ignore headers
	if (/^==.+==\s*$/) { next; }
	
	# remove quotes
	s/&quot;//g;

	# Kill image tags of the form [[Image:Chemin.png|thumb|300px|blah]]
	s/\[\[Image:.+?\]\]//g;

	# Kill File tags of the form [[File:blah.jpg|thumb|right|350px| blah]]
	s/\[\[File:.+?\]\]//g;

	# kill wikilinks of the form [[the real link#ugh|The Stand-In Text]]
	# also [[Wikipedia:special/blah|The Stand-In Text]]
	s/\[\[[\p{Word}\p{Space}#:,–“„”‘!&€\.\/\-\$\*\w '\(\)]+?\|(.*?)\]\]/$1/g;

	# Kill ordinary links -- [[Stuff more stuff]]
	s/\[\[([\p{Word}\p{Space}:,–“„”‘!&€\.\/\-\+\$\*\w '\(\)]+?)\]\]/$1/g;

	# kill weblinks  i.e. [http:blah.com/whatever A Cool Site]
	s/\[\S+ (.+?)\]/$1/g;

	# kill weblinks with no text i.e. [http:blah.com/whatever]
	s/\[http(.+?)\]//g;

	# ignore misc html markup
	s/&lt;references\s*\/&gt;//g;
	s/&lt;i&gt;//g;
	s/&lt;i .+?&gt;//g;
	s/&lt;\/i&gt;//g;
	s/&lt;p&gt;//g;
	s/&lt;p .+?&gt;//g;
	s/&lt;\/p&gt;//g;
	s/&lt;b&gt;//g;
	s/&lt;b .+?&gt;//g;
	s/&lt;\/b&gt;//g;
	s/&lt;s&gt;//g;
	s/&lt;\/s&gt;//g;
	s/&lt;u&gt;//g;
	s/&lt;\/u&gt;//g;
	s/&lt;em&gt;//g;
	s/&lt;\/em&gt;//g;
	s/&lt;tt&gt;//g;
	s/&lt;\/tt&gt;//g;
	s/&lt;title&gt;//g;
	s/&lt;\/title&gt;//g;
	s/&lt;comment&gt;//g;
	s/&lt;\/comment&gt;//g;
	s/&lt;username&gt;//g;
	s/&lt;\/username&gt;//g;
	s/&lt;timestamp&gt;//g;
	s/&lt;\/timestamp&gt;//g;
	s/&lt;id&gt;//g;
	s/&lt;\/id&gt;//g;
	s/&lt;pre&gt;//g;
	s/&lt;pre .+?&gt;//g;
	s/&lt;\/pre&gt;//g;
	s/&lt;big&gt;//g;
	s/&lt;\/big&gt;//g;
	s/&lt;small&gt;//g;
	s/&lt;\/small&gt;//g;
	s/&lt;center&gt;//g;
	s/&lt;Center&gt;//g;
	s/&lt;\/center&gt;//g;
	s/&lt;inputbox&gt;//g;
	s/&lt;\/inputbox&gt;//g;
	s/&lt;charinsert&gt;//g;
	s/&lt;\/charinsert&gt;//g;
	s/&lt;timeline&gt;//g;
	s/&lt;\/timeline&gt;//g;
	s/&lt;cite&gt;//g;
	s/&lt;cite .+?&gt;//g;
	s/&lt;\/cite&gt;//g;
	s/&lt;Cite&gt;//g;
	s/&lt;Cite .+?&gt;//g;
	s/&lt;\/Cite&gt;//g;
	s/&lt;blockquote&gt;//g;
	s/&lt;\/blockquote&gt;//g;
	s/&lt;div .+?&gt;//g;
	s/&lt;\/div&gt;//g;
	s/&lt;font .+?&gt;//g;
	s/&lt;\/font&gt;//g;
	s/&lt;FONT .+?&gt;//g;
	s/&lt;\/FONT&gt;//g;
	s/&lt;span .+?&gt;//g;
	s/&lt;\/span&gt;//g;
	s/&lt;h1&gt;//g;
	s/&lt;h1 .+?&gt;//g;
	s/&lt;\/h1&gt;//g;
	s/&lt;h2&gt;//g;
	s/&lt;h2 .+?&gt;//g;
	s/&lt;\/h2&gt;//g;
	s/&lt;h3&gt;//g;
	s/&lt;h3 .+?&gt;//g;
	s/&lt;\/h3&gt;//g;
	s/&lt;h4&gt;//g;
	s/&lt;h4 .+?&gt;//g;
	s/&lt;\/h4&gt;//g;
	s/&lt;br&gt;//g;
	s/&lt;BR&gt;//g;
	s/&lt;\/br&gt;//g;
	s/&lt;br\/&gt;//g;
	s/&lt;br \/&gt;//g;
	s/&lt;br .*?&gt;//g;
	s/&lt;hr&gt;//g;
	s/&lt;hr .+?&gt;//g;
	s/&lt;includeonly&gt;//g;
	s/&lt;\/includeonly&gt;//g;
	s/&lt;noinclude&gt;//g;
	s/&lt;\/noinclude&gt;//g;
	s/&lt;Typo .+?\/&gt;//g;
	s/&lt;nowiki&gt;//g;
	s/&lt;nowiki \/&gt;//g;
	s/&lt;\/nowiki&gt;//g;
	s/__NOTOC__//g;
	s/&lt;ul&gt;//g;
	s/&lt;\/ul&gt;//g;
	s/&lt;li&gt;//g;
	s/&lt;li .?&gt;//g;
	s/&lt;\/li&gt;//g;
	s/&lt;tr&gt;//g;
	s/&lt;tr .+?&gt;//g;
	s/&lt;\/tr&gt;//g;
	s/&lt;td&gt;//g;
	s/&lt;td .+?&gt;//g;
	s/&lt;\/td&gt;//g;
	s/&lt;\/td .+?&gt;//g;

	# restore ordinary markup
	s/&amp;/&/g;
	s/&ndash;/-/g;
	# s/&minus;/-/g;
	s/&lt;/</g;
	s/&gt;/>/g;
	s/&deg;/°/g;
	s/&bull;/•/g;
	s/&nbsp;/ /g;

	# Make sure bulleted lists have a period at the end of them.
	# But do try to avoid double-periods.
	# Also don't add the period if it is empty.
	if (/^\*\S+/ || /^#\S+/ || /^:\S+/ || /^-\S+/ || /^–\S+/) {
		# Ignore }} append at the end, if any
		# e.g. * Sommerdahl}}
		s/\}+$//;

		if (!/\.$/) { $_ = $_ . "."; }
	}

	# kill bullets
	s/^\*\*\*//;
	s/^\*\*//;
	s/^\*//;
	s/^#//;
	s/^:::::://;
	s/^::::://;
	s/^:::://;
	s/^::://;
	s/^:://;
	s/^://;
	s/^-+//;

	# Ignore plain }} lines
	s/^\s*\}+$//;

	# Ignore plain ]] lines
	s/^\s*\]+$//;

	# Trim
	s/^\s+|\s+$//;

	if ($page_not_open) {
		$page_not_open = 0;
		open PAGE, ">" . $page_out_directory . "/" . $page_title;
		binmode PAGE, ':encoding(UTF-8)';
	}
	if (length $_) { print PAGE "$_\n"; }
}
