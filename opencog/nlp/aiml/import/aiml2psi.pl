#! /usr/bin/env perl
#
# Convert AIML files to OpenCog Atomese.
#
# The perl script converts AIML XML into OpenCog OpenPsi rules.  See the
# bottom for an example of the output format, and a breif discussion
# about the design choices taken.
#
# The use of AIML is strongly discouraged, and is not a formal OpenCog
# project goal. However, there are various requests from various forces
# asking for AIML-within-OpenCog capabilities, and this script is meant
# to fulfill these requests.
#
# Copyright (c) Kino Coursey 2015
# Copyright (c) Linas Vepstas 2016
#
use Getopt::Long qw(GetOptions);
use strict;

my $ver = "0.5.7";
my $debug;
my $help;
my $version;
my $overwrite = 0;
my $aimlDir ='.';
my $intermediateFile = 'aiml-flat.txt';
my $outDir = '';
my $outFile = 'aiml-rules.scm';
my $weightFile = '';

my $base_priority = 1.0;

my $cmdline = $0 . " " . join(" ", @ARGV);

GetOptions(
    'dir=s' => \$aimlDir,
    'debug' => \$debug,
    'help' => \$help,
    'last-only' => \$overwrite,
    'version' => \$version,
    'intermediate=s' => \$intermediateFile,
    'out=s' => \$outDir,
    'outfile=s' => \$outFile,
    'priority=f' => \$base_priority,
    'weights=s' => \$weightFile,
) or die "Usage: $0 [--debug] [--help] [--version] [--last-only] [--dir <AIML source directory>] [--intermediate <IMMFile>] [--out <output directory>] [--outfile <filename>] [--weights <weight-filename>]\n";

if ($help)
{
	print "Convert AIML markup files to OpenCog Atomese files.\n";
	print "\n";
	print "Usage: $0 [--debug] [--help] [--version] [--last-only] [--dir <AIML source directory>] [--intermediate <IMMFile>] [--out <OpenCog file>]\n";
	print "   --debug                 Enable debugging (if any).\n";
	print "   --help                  Print these helpful comments.\n";
	print "   --version               Print version, current version '$ver'\n";
	print "   --last-only             Only the last category is output.\n";
	print "   --dir <directory>       AIML source directory, default: '$aimlDir'\n";
	print "   --intermediate <file>   Intermediate file, default: '$intermediateFile'\n";
	print "   --out <directory>       Dir for many small output files.\n";
	print "   --outfile <filename>    Output one large file, default: '$outFile'\n";
	print "   --weights <filename>    Input file, holding rule entropies\n";
	print "   --priority <float>      Rule priority, default: '$base_priority'\n";
	die "\n";
}

if ($version)
{
	print "version $ver\n";
	die "\n";
}

# ------------------------------------------------------------------
# If there is a weights file, ingest it, and stuff it into an associate
# array.
#
# The file format is assumed to be three columns: an AIML filename,
# an AIML rule, and a log-liklihood column.
#
# Example:
#
# interjection.aiml	INTERJECTION <THAT> * <TOPIC> *	-1.97356795842974
# stack.aiml	PUSH * <THAT> * <TOPIC> *	-2.08673772127076
# mp0.aiml	INSULT <THAT> * <TOPIC> *	-3.96370879747086
# default.aiml	IT * <THAT> * <TOPIC> *	-4.12722757190359
# that.aiml	THAT * <THAT> * <TOPIC> *	-4.25883896393828
# atomic.aiml	WHY <THAT> * <TOPIC> *	-4.28192456627784
#

my %weights = ();
sub make_wkey
{
	my $key = $_[0] . " <THAT> " . $_[1] . " <TOPIC> " . $_[2];
}

sub trim { my $s = shift; $s =~ s/^\s+|\s+$//g; return $s };

sub ingest_weights
{
	if ('' eq $weightFile)
	{
		print "No weightfile specified.\n";
		return;
	}
	open WFILE, $weightFile
		or die "Can't open the weight file `$weightFile`\n";
	print "Reading weights from `$weightFile`\n";

	while (<WFILE>)
	{
		chop;
		# split into filename, text, log-liklihood
		if (/^([\w\.]+?\.aiml)\s+(.*)\s+-([\d\.]+)\s*$/)
		{
			my $filename = $1;
			my $loglikeli = $3;
			my $mlpat = $2;

			# Extract the AIML pattern, the THAT and the TOPIC
			if ($mlpat =~ /^(.*)\s*<THAT>\s*(.*?)\s*<TOPIC>\s*(.*)\s*$/)
			{
				my $pat = $1;
				my $that = $2;
				my $topic = $3;
				# $pat = lc $pat;
				$pat = trim $pat;

				# my $key = $pat . " <THAT> " . $that . " <TOPIC> " . $topic;
				my $key = make_wkey($pat, $that, $topic);
				$weights{$key} = $loglikeli;
			}
			else
			{
				print "Unexpected format in the weights file: >>>$_<<<\n";
			}
		}
		else
		{
			print "Unexpected text in the weights file: >>>$_<<<\n";
		}
	}
	close WFILE;
}

&ingest_weights();

# ------------------------------------------------------------------
# Conversion is done in a two-pass process.  The first pass flattens
# the AIML format into a simplified linear format.  A second pass
# converts this flattened format into Atomese.

print "\nAIML Source directory = $aimlDir\n";
opendir(DIR, "$aimlDir");
my @aimlFiles = grep(/\.aiml$/, readdir(DIR));
closedir(DIR);

open FOUT, ">$intermediateFile";
foreach my $af (sort @aimlFiles)
{
	my $textfile="";
	my $aimlSrc = "$aimlDir/$af";
	print " \n\n*****  processing $aimlSrc ****\n";
	# read the entire file in as one string
	open FILE, "$aimlSrc" or die "Couldn't open file: $!";
	while (<FILE>) {
		$textfile .= $_;
	}
	close FILE;
	$textfile .="\n";


	# Goal: read AIML into a linear neutral format while preserving
	# relevant semantic info, such as the order of pattern side slot
	# filling stars or sets

	my $topicx = "*";

	# Normalize file by removing line feeds and excess spaces.
	$textfile =~ s/\r\n/ /gi;
	$textfile =~ s/\n/ /gi;
	$textfile =~ s/\r/ /gi;
	$textfile =~ s/ xml\:space=\"preserve\"//gi;
	$textfile =~ s/ xml\:space=\"default\"//gi;

	while ($textfile =~ /  /) { $textfile =~ s/  / /gi;}

	# Normalize so that every category has a pattern/topic/that/template
	# entries.
	$textfile =~ s/\<\/pattern\> \<template\>/\<\/pattern\> \<that\>*\<\/that\> \<template\>/gi;

	# Define where to split for analysis.
	$textfile =~ s/<category>/\#\#SPLIT \<category\>/gi;
	$textfile =~ s/<\/category>/\<\/category\>\#\#SPLIT /gi;
	$textfile =~ s/<topic /\#\#SPLIT\<topic /gi;
	$textfile =~ s/<\/topic>/\<\/topic\>\#\#SPLIT /gi;
	$textfile =~ s/<aiml/\#\#SPLIT\<aiml/gi;
	$textfile =~ s/<\/aiml>/\<\/aiml\>\#\#SPLIT /gi;

	my @cats = split(/\#\#SPLIT/, $textfile);

	# It should be one category at a time, but it could be on high-level
	# topics.
	foreach my $c (@cats)
	{
		# print FOUT "$c\n";
		# Processing high level topic conditions.
		if ($c =~ /<topic /)
		{
			my @t = $c =~ /name=\"(.*?)\"/;
			$topicx = $t[0];
			next;
		}
		if ($c =~ /<\/topic>/)
		{
			$topicx = "";
			next;
		}

		# Processing general categories.
		if ($c =~ /<category>/)
		{
			my $path="";
			if ($c !~ /<topic>/)
			{
				my $tpat = "\<\/pattern\> \<topic\>". $topicx ."\<\/topic\>";
				$c =~ s/\<\/pattern\>/$tpat/;
			}
			my @pat = $c =~ m/\<pattern\>(.*?)\<\/pattern\>/;
			my @top = $c =~ m/\<topic\>(.*?)\<\/topic\>/;
			my @that = $c =~ m/\<that\>(.*?)\<\/that\>/;
			my @template = $c =~ m/\<template\>(.*?)\<\/template\>/;
			if (@pat == 0) { next; }
			if (@template == 0) { next; }
			if (@that == 0) { push(@that,""); }
			if (@top == 0) { push(@top,""); }

			# Special cases.
			#	pattern side <set>{NAME}</set> and <bot name=""/>
			#
			if (@pat >0) {$pat[0]=~ s/\<bot name/\<bot_name/gi; }
			if (@pat >0) {$pat[0]=~ s/\<set> /<set>/gi; }
			if (@top >0) {$top[0]=~ s/\<set> /<set>/gi; }
			if (@that >0) {$that[0]=~ s/\<set> /<set>/gi; }

			if (@pat >0)  {$pat[0]=~ s/ <\/set>/<\/set>/gi; }
			if (@top >0)  {$top[0]=~ s/ <\/set>/<\/set>/gi; }
			if (@that >0) {$that[0]=~ s/ <\/set>/<\/set>/gi; }

			my @PWRDS = split(/ /,$pat[0]);
			my @TWRDS = split(/ /,$that[0]);
			my @TPWRDS = split(/ /,$top[0]);
			my $pstars=0;
			my $tstars=0;
			my $topicstars=0;

			print FOUT "CATBEGIN,0\n";
			print FOUT "CATTEXT,$c\n";

			# Patterns.
			print FOUT "PAT,$pat[0]\n";
			$path .="<input>";
			foreach my $w (@PWRDS)
			{
				$path .="/$w";
				if ($w eq "*")
				{
					$pstars++;
					print FOUT "PSTAR,$pstars\n";
					next;
				}
				if ($w eq "_")
				{
					$pstars++;
					print FOUT "PUSTAR,$pstars\n";
					next;
				}
				if ($w =~ /<set>/)
				{
					my @set = $w =~ /<set>(.*?)<\/set>/;
					print FOUT "PSET,$set[0]\n";
					next;
				}
				if ($w =~ /<bot_name/)
				{
					my @v = $w =~ /name=\"(.*?)\"/;
					print FOUT "PBOTVAR,$v[0]\n";
					next;
				}

				print FOUT "PWRD,$w\n";
			}
			print FOUT "PATEND,0\n";

			# Topics
			print FOUT "TOPIC,$top[0]\n";
			$path .="/<topic>";
			foreach my $w (@TPWRDS)
			{
				$path .="/$w";
				if ($w eq "*")
				{
					$topicstars++;
					print FOUT "TOPICSTAR,$topicstars\n";
					next;
				}
				if ($w eq "_")
				{
					$topicstars++;
					print FOUT "TOPICUSTAR,$topicstars\n";
					next;
				}
				if ($w =~ /<set>/)
				{
					my @set = $w =~ /<set>(.*?)<\/set>/;
					print FOUT "TOPICSET,$set[0]\n";
					next;
				}
				if ($w =~ /<bot_name/)
				{
					my @v = $w =~ /name=\"(.*?)\"/;
					print FOUT "TOPICBOTVAR,$v[0]\n";
					next;
				}
				print FOUT "TOPICWRD,$w\n";
			}
			print FOUT "TOPICEND,0\n";

			# That
			print FOUT "THAT,$that[0]\n"; #
			$path .="/<that>";
			foreach my $w (@TWRDS)
			{
				$path .="/$w";
				if ($w eq "*")
				{
					$tstars++;
					print FOUT "THATSTAR,$tstars\n";
					next;
				}
				if ($w eq "_")
				{
					$tstars++;
					print FOUT "THATUSTAR,$tstars\n";
					next;
				}
				if ($w =~ /<set>/)
				{
					my @set = $w =~ /<set>(.*?)<\/set>/;
					print FOUT "THATSET,$set[0]\n";
					next;
				}
				if ($w =~ /<bot_name/)
				{
					my @v = $w =~ /name=\"(.*?)\"/;
					print FOUT "THATBOTVAR,$v[0]\n";
					next;
				}
				print FOUT "THATWRD,$w\n";
			}
			print FOUT "THATEND,0\n";

			# Templates.
			# Use AIMLIF convention of escaping sequences that are not CSV
			# compliant namely ","-> "#Comma "
			if ( @template > 0)
			{
				$template[0] =~ s/\,/\#Comma /gi;
				$template[0] =~ s/^ //gi;
				$template[0] =~ s/ $//gi; #
				print FOUT "PATH,$path\n";

				# Will probably have to expand this a bit,
				# since it requires representing the performative
				# interpretation of XML that AIML assumes.
				if ($template[0] !~ /</) #
				{
					# Remove HTML-encoded XML. This is mostly going to be
					# XML meant to control some text-to-speech system.
					my $raw = $template[0];
					while ($raw =~ /(.*)&lt;(.+?)&gt;(.*)/)
					{
						$raw = $1 . $3;
					}

					# Space-pad embedded long dashes.
					$raw =~ s/---/ --- /g;
					$raw =~ s/--/ -- /g;

					print FOUT "TEMPATOMIC,0\n";
					my @TEMPWRDS = split(/ /, $raw); #
					foreach my $w (@TEMPWRDS)
					{
						if (length($w)>0)
						{
							print FOUT "TEMPWRD,$w\n";
						}
					}
					print FOUT "TEMPATOMICEND,0\n";
				}
				else
				{
					print FOUT "TEMPLATECODE,$template[0]\n";
				}
			}
			else
			{
				print FOUT "TEMPLATECODE,$template[0]\n";
			}

			print FOUT "TEMPLATE,$template[0]\n";
			print FOUT "CATEND,0\n";
			print FOUT "\n";
		}
	}
}
close(FOUT);

# ------------------------------------------------------------------
# Second pass utilities

my $star_index = 1;  # First star has index of one.
my $do_count_stars = 0;  # do not count stars, if this is not set.
my $word_count = 0;
my $pat_word_count = 0;

my $wordnode = "(Word ";
# my $wordnode = "(Concept ";

sub trim_punct
{
	my $wrd = $_[0];

	# Remove whitespace.
	$wrd =~ s/\s*//;

	# More HTML markup is sneaking by...
	$wrd =~ s/&gt;//g;

	# Remove leading and trailing punctuation, keep star and underscore.
	# Keep embedded dots (for decimal numbers!?, acronyms, abbreviations)
	# Keep exclamation and question mark, maybe the text-to-speech can do
	# something with that?
	# $wrd =~ s/^[.'(){}\-:;!?,"\\\/<>]+//;
	$wrd =~ s/^[.'(){}\-:;,"\\\/<>]+//;
	$wrd =~ s/[.'(){}\-:;,"\\\/<>]+$//;

	# Remove back-slashed quotes in the middle of words.
	$wrd =~ s/\.\\"//g;
	$wrd =~ s/\\"//g;

	# Convert any remaining backslashes into forward-slashes.
	$wrd =~ s/\\/\//g;

	$wrd;
}

# split_string -- split a string of words into distinct nodes.
sub split_string
{
	my $indent = $_[0];
	my $text = $_[1];
	my @words = split(/ /, $text);
	my $tout = "";
	for my $wrd (@words)
	{
		# Remove punction.
		$wrd = &trim_punct($wrd);

		if ($wrd eq "") {}
		elsif ($wrd eq "*" or $wrd eq "_")
		{
			$tout .= $indent . "(Glob \"\$star-$star_index\")\n";
			if (0 < $do_count_stars) { $star_index ++; }
		}
		else
		{
			$tout .= $indent . $wordnode . "\"$wrd\")\n";
			$word_count ++;
		}
	}
	$tout;
}

sub process_aiml_tags;

# process_star -- star extraction
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_star
{
	my $indent = $_[0];
	my $text = $_[1];

	my $tout = "";
	$text =~ /(.*?)<star(.*)/;
	$tout .= &split_string($indent, $1);

	my $star = $2;
	$star =~ s/^\s*//;
	$star =~ s/\s*$//;
	$star =~ s/\\'/'/g;
	# Handle both <star index='1'/> and <star index='1'></star>
	if ($star =~ /^index='(\d+)'(\s*\/>|>\s*<\/star>)(.*)/)
	{
		$tout .= $indent . "(Glob \"\$star-$1\")\n";

		my $t = $3;
		$t =~ s/^\s*//;
		$t =~ s/\s*$//;
		if ($t ne "")
		{
			$tout .= &process_aiml_tags($indent, $t);
		}
	}
	elsif ($star =~ /^\/>(.*)/)
	{
		$tout .= $indent .  "(Glob \"\$star-1\")\n";
		my $t = $1;
		$t =~ s/^\s*//;
		$t =~ s/\s*$//;
		if ($t ne "")
		{
			$tout .= &process_aiml_tags($indent, $t);
		}
	}
	else
	{
		print "Ohhhh nooo, Mr. Bill!\n";
		print "$text\n";
		die;
	}
	$tout;
}

# process_tag -- process a generic, un-named tag
#
# First argument: the tag name
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $text = $_[2];
	my $tout = "";

	$text =~ /(.*?)<$tag>(.*?)<\/$tag>(.*)/;

	my $t1 = $1;
	my $t2 = $2;
	my $t3 = $3;

	$tout .= &process_aiml_tags($indent, $t1);
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag $tag\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (ListLink\n";
	$tout .= &process_aiml_tags($indent . "         ", $t2);
	$tout .= $indent . "   )))\n";
	if ($t3 ne "")
	{
		$tout .= &process_aiml_tags($indent, $t3);
	}

	$tout;
}

# process_set -- process a set tag
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_set
{
	my $indent = $_[0];
	my $text = $_[1];
	my $tout = "";

	$text =~ /(.*?)<set name='(.*?)'>(.*?)<\/set>(.*)/;

	my $t1 = $1;
	my $t2 = $2;
	my $t3 = $3;
	my $t4 = $4;

	# For nested <set> tags like:
	# "<set name='it'> <set name='topic'> test </set> </set>"
	# $3 will be "<set name='topic'> test" using the above regex,
	# which is invalid as it doesn't include the "</set>"
	if (index($t3, "<set name") != -1)
	{
		# For handling nested <set> tag
		$text =~ /(.*?)<set name='(.*?)'>(.*)<\/set>(.*)/;

		$t1 = $1;
		$t2 = $2;
		$t3 = $3;
		$t4 = $4;
	}

	$tout .= &split_string($indent, $t1);
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag set\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (Concept \"" . $t2 . "\")\n";
	$tout .= $indent . "      (ListLink\n";
	$tout .= &process_aiml_tags($indent . "         ", $t3);
	$tout .= $indent . "   )))\n";
	if ($t4 ne "")
	{
		$tout .= &process_aiml_tags($indent, $t4);
	}
	$tout;
}

# Print out a tag schema for named tag
#
# First argument: the tag name
# Second argument: white-space indentation to insert on each line.
# Third argument: the value for the tag.
sub print_named_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $arg = $_[2];
	my $tout = "";
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag $tag\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (Concept \"$arg\")\n";
	$tout .= $indent . "   ))\n";
	$tout;
}

# Print out a tag predicate for named tag
#
# First argument: the tag name
# Second argument: white-space indentation to insert on each line.
# Third argument: the value for the tag.
sub print_named_eval_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $arg = $_[2];
	my $tout = "";
	$tout .= $indent . "(EvaluationLink\n";
	$tout .= $indent . "   (DefinedPredicate \"AIML-pred $tag\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (Concept \"$arg\")\n";
	$tout .= $indent . "   ))\n";
	$tout;
}

# Print out an Evaluation (predicate) pattern
#
# First argument: the tag name
# Second argument: white-space indentation to insert on each line.
# Third argument: the value for the tag.
sub print_predicate_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $arg = $_[2];
	my $anchor = $tag;

	if ($tag eq "pattern")
	{
		$anchor = "*-AIML-pattern-*";
		$do_count_stars = 1;
		$star_index = 1;
	}
	elsif ($tag eq "that")
	{
		$anchor = "*-AIML-that-*";
		$do_count_stars = 1;
		$star_index = 1;
	}
	elsif ($tag eq "topic")
	{
		$anchor = "*-AIML-topic-*";
		$do_count_stars = 1;
		$star_index = 1;
	}
	my $tout = "";
	$tout .= $indent . "(Evaluation\n";
	$tout .= $indent . "   (Predicate \"$anchor\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= &process_aiml_tags($indent . "      ", $arg);
	$tout .= $indent . "   ))\n";

	$do_count_stars = 0;
	$star_index = 1;
	$tout;
}

# process_named_tag -- process a generic tag that has a name
#
# First argument: the tag name
# Second argument: white-space indentation to insert on each line.
# Third argument: the actual text to unpack
sub process_named_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $text = $_[2];
	my $tout = "";

	$text =~ /(.*?)<$tag name='(.*?)'\/>(.*)/;

	my $t1 = $1;
	my $t2 = $2;
	my $t3 = $3;

	$tout .= &split_string($indent, $t1);
	$tout .= &print_named_tag($tag, $indent, $t2);
	$tout .= &process_aiml_tags($indent, $t3);
	$tout;
}

# process_that -- process a that tag
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_that
{
	my $indent = $_[0];
	my $text = $_[1];
	my $idx = 1;
	my $tout = "";

	# For example, <that/>, <that index="1,1"/> and <that index="2"/>
	# index is optional, 2nd dimension of the index is ignored as we
	# don't support it right now and nobody is really using it
	$text =~ /(.*?)<that( index\s*=\s*'(\d+)(.*)?')?\s*\/>(.*)/;

	$tout .= &split_string($indent, $1);
	if ($3 ne "")
	{
		$idx = $3;
	}
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag that\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (Number \"$idx\")))\n";
	if ($5 ne "")
	{
		$tout .= &process_aiml_tags($indent, $5);
	}
	$tout;
}

my @all_choices = ();

# process_random -- process a random tag
#
# First argument: the to-be-processed text that contains random tags
# Second argument: the CATTEXT
# Third argument: number of stars in the pattern
# Fourth argument: the context of the psi-rule
sub process_random
{
	my $rules = "";
	my $raw_code = $_[0];
	my $cattext = $_[1];
	my $num_stars = $_[2];
	my $psi_ctxt = $_[3];

	&discover_choices($raw_code);

	my $i = 1;
	my $num_choices = $#all_choices + 1;
	foreach my $catty (@all_choices)
	{
		my $wadj = &get_weight($cattext);

		$rules .= ";;; random choice $i of $num_choices: ";
		$rules .= $cattext . "\n";
		$rules .= "(psi-rule\n";
		$rules .= "   ; context\n";
		$rules .= $psi_ctxt;
		$rules .= "   ; action\n";
		$rules .= "   (ListLink\n";
		$rules .= &process_category("      ", $catty);
		$rules .= "   )\n";
		$rules .= &psi_tail($num_stars, $pat_word_count, $num_choices, $wadj);
		$rules .= ") ; random choice $i of $num_choices\n\n";  # close category section
		$i = $i + 1;
	}

	# Reset @all_choices
	@all_choices = ();

	$rules;
}

# discover_choices -- unpack the random tag and get the items
#
# First argument: text that contains random tags
sub discover_choices
{
	my $text = $_[0];
	$text =~ /(.*?)<random>(.*?)<\/random>(.*)/;

	my $t1 = $1;
	my $t2 = $2;
	my $t3 = $3;

	# $t1 should never has any random tag
	if ($t1 ne "")
	{
		my @one = ($t1);
		&generate_choices(\@one);
	}

	# If $t2 has a random tag, they are probably nested random tags
	if (index($t2, "<random>") != -1)
	{
		$text =~ /(.*?)<random>(.*)<\/random>(.*)/;

		# Update $t2 and $t3
		$t2 = $2;
		$t3 = $3;

		# XXX TODO: This removes all of the nested random tags and
		# hence is treating those elements as if they were under the
		# same random tag. May need to correct the weight?
		# Also this is wrong as it ignores the text between <li> &
		# <random>, and </li> & </random>, if any, but it doesn't seem
		# to be a big deal in our use case at the moment, would be better
		# to actually do recursive calls
		$t2 =~ s/<li>.*?<random>//g;
		$t2 =~ s/<\/random>.*?<\/li>//g;
	}

	$t2 =~ s/^\s+//;
	my @choicelist = split /<li>/, $t2;

	foreach my $ch (@choicelist)
	{
		$ch =~ s/<\/li>//;
		$ch =~ s/\s+$//;
	}

	# Remove empty elements
	@choicelist = grep($_, @choicelist);
	&generate_choices(\@choicelist);

	# If $t3 has a random tag, it means there are more than one
	# random tags
	if (index($t3, "<random>") != -1)
	{
		&discover_choices($t3);
	}
	elsif ($t3 ne "")
	{
		my @three = ($t3);
		&generate_choices(\@three);
	}
}

# generate_choices -- generate the combinations
#
# First argument: a list of items from a random tag
sub generate_choices
{
	my @choices = @{$_[0]};
	my @new_choices = ();

	foreach my $ac (@all_choices)
	{
		foreach my $ch (@choices)
		{
			if ($ch ne "")
			{
				push(@new_choices, ($ac . $ch));
			}
		}
	}

	if ($#all_choices == -1)
	{
		@all_choices = @choices;
	}
	else
	{
		@all_choices = @new_choices;
	}
}

# process_input -- process a input tag
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_input
{
	my $indent = $_[0];
	my $text = $_[1];
	my $idx = 1;
	my $tout = "";

	# For example, <input/> and <input index="2">
	$text =~ /(.*?)<input( index\s*=\s*'(\d+)')?\s*\/>(.*)/;

	$tout .= &split_string($indent, $1);
	if ($3 ne "")
	{
		$idx = $3;
	}
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag input\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "       (Number \"$idx\")))\n";
	if ($4 ne "")
	{
		$tout .= &process_aiml_tags($indent, $4);
	}
	$tout;
}

# process_category -- convert AIML <category> into Atomese.
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_category
{
	my $indent = $_[0];
	my $text = $_[1];

	# lower-case everything
	$text = lc $text;

	# Expand defintion of <sr/>
	$text =~ s/<sr\/>/<srai><star\/><\/srai>/g;
	$text =~ s/<sr \/>/<srai><star\/><\/srai>/g;
	$text =~ s/<srai \/>/<srai><star\/><\/srai>/g;

	# typo
	$text =~ s/<peron/<person/g;
	$text =~ s/<\/peron/<\/person/g;
	$text =~ s/<thastar/<thatstar/g;

	# XXX FIXME ? This is supposed to be equivalent to
	# <person><star/></person> however, in the actual AIML texts,
	# there is no actual star, so its broken/invalid sytax.
	$text =~ s/<person\/>/<person><star\/><\/person>/g;
	$text =~ s/<person \/>/<person><star\/><\/person>/g;
	$text =~ s/<person2\/>/<person2><star\/><\/person2>/g;
	$text =~ s/<person2 \/>/<person2><star\/><\/person2>/g;

	# Convert mangled commas, from pass 1
	$text =~ s/#Comma/,/g;

	# Unescape escaped single-quotes.
	$text =~ s/\\'/'/g;

	# Escape back-slashes
	$text =~ s/\\/\\\\/g;

	# strip out HTML markup. <a href> tag
	$text =~ s/<a (target|href)=.*?>//g;
	$text =~ s/<\/a>//g;
	$text =~ s/<ul>//g;
	$text =~ s/<\/ul>//g;
	$text =~ s/<li>//g;
	$text =~ s/<\/li>//g;
	$text =~ s/<uppercase>//g;
	$text =~ s/<\/uppercase>//g;
	$text =~ s/<p\/>//g;
	$text =~ s/<img src=.*?>//g;
	$text =~ s/<\/img>//g;
	$text =~ s/<property.*?>//g;
	$text =~ s/<id\/>//g;
	$text =~ s/<id>\s*<\/id>//g;
	$text =~ s/<br\/>//g;
	$text =~ s/<em>//g;
	$text =~ s/<\/em>//g;

	# Backward compatible with '<get_*', turn it into <get name='*'/>
	$text =~ s/<get_(.*?)(\s*\/>|>\s*<\/get_.*?>)/<get name='$1'\/>/g;

	# Trim leading and trailing whtespace.
	$text =~ s/^\s*//;
	$text =~ s/\s*$//;

	my $tout = &process_aiml_tags($indent, $text);
	$tout;
}

# process_aiml_tags -- convert AIML tags into Atomese.
# Currently handles STAR and SRAI.
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_aiml_tags
{
	my $indent = $_[0];
	my $text = $_[1];

	if ($text eq "") { return ""; }

	my $tout = "";

	# Find the very first angle bracket
	if ($text =~ /(.*?)<(.*)/)
	{
		my $preplate = $1;
		my $tag = $2;

		if ($tag =~ /^srai>/)
		{
			$tout .= &process_tag("srai", $indent, $text);
		}
		elsif ($tag =~ /^star/)
		{
			$tout .= &process_star($indent, $text);
		}
		elsif ($tag =~ /^think>/)
		{
			$tout .= &process_tag("think", $indent, $text);
		}
		elsif ($tag =~ /^set name/)
		{
			$tout .= &process_set($indent, $text);
		}
		elsif ($tag =~ /^person>/)
		{
			$tout .= &process_tag("person", $indent, $text);
		}
		elsif ($tag =~ /^person2>/)
		{
			$tout .= &process_tag("person2", $indent, $text);
		}
		elsif ($tag =~ /^person.*>(.*?)/)
		{
			print "Aieee! Unhandled screwball person tag!!!\n";
			print "$text\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^that/)
		{
			$tout .= &process_that($indent, $text);
		}
		elsif ($tag =~ /^input/)
		{
			$tout .= &process_input($indent, $text);
		}
		elsif ($tag =~ /^get name/)
		{
			$tout .= &process_named_tag("get", $indent, $text);
		}
		elsif ($tag =~ /^bot name/)
		{
			$tout .= &process_named_tag("bot", $indent, $text);
		}
		elsif ($tag =~ /^formal>/)
		{
			$tout .= &process_tag("formal", $indent, $text);
		}
		elsif ($tag =~ /^!--.*-->(.*)/)
		{
			# WTF is <!-- REDUCTION --> ??? whatever it is we don't print it.
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^(.*?)&gt;(.*)/)
		{
			# These occur when the responses are trying to explain XML.
			# It creates a huge mess, so blow it all off.
			#$tout .= &split_string($indent, $preplate);
			#$tout .= &process_aiml_tags($indent, "greater " . $1 . " less " . $2);
		}
		elsif ($tag =~ /^date.*?>(.*)/)
		{
			# These are harder to handle and we don't use them so screw it.
			print "Aieee! Unhandled date tag!!!\n";
			print "See file line number $.\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^size\/>(.*)/)
		{
			# Blow this off.
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^get_likes.*?>(.*)/)
		{
			# WTF is this???
			print "Aieee! weird stuff!!!\n";
			print "See file line number $.\n";
			print "$text\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^random>/)
		{
			# These are harder to handle and we don't use them so screw it.
			print "Aieee! Nested random tag!!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		elsif ($tag =~ /^\/random>/)
		{
		}
		elsif ($tag =~ /^\/set>/)
		{
			# Sometimes, recursion screws up. This is rare, and I'm going
			# to punt, for now.
			print "Aieee! Bad recursion!!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		elsif ($tag =~ /^\/think>/)
		{
			# Sometimes, recursion screws up. This is rare, and I'm going
			# to punt, for now.
			print "Aieee! Bad recursion!!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		elsif ($tag =~ /^\/srai>/)
		{
			# Sometimes, recursion screws up. This is rare, and I'm going
			# to punt, for now.
			print "Aieee! Bad recursion!!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		elsif ($tag =~ /^condition/)
		{
			# WTF. Blow this off, for now.
			print "Aieee! Condition tag is not handled!!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		elsif ($tag =~ /^\/condition>/)
		{
		}
		elsif ($tag =~ /^topicstar\/>/)
		{
			# WTF. Blow this off, for now.
			print "Aieee! topicstar tag is not handled!!!\n";
			print "See file line number $.\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^thatstar\/>/)
		{
			# WTF. Blow this off, for now.
			print "Aieee! thatstar tag is not handled!!!\n";
			print "See file line number $.\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^bot_name/)
		{
			# Blow this off
			print "Aieee! bot_name tag in the pattern!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
			$tout .= &process_aiml_tags($indent, $preplate . " " . $1);
		}
		elsif ($tag =~ /^that/)
		{
			# Blow this off
			print "Aieee! Wacky that tag!!\n";
			print "See file line number $.\n";
			print ">>>>>>$text\n";
		}
		else
		{
			print "Aieee! what is this tag???\n";
			print "See file line number $.\n";
			print ">>>>>>$tag\n\n\n";
			print ">>>>>>$text\n";
			die;
		}
	}
	else
	{
		$tout .= &split_string($indent, $text);
	}
	$tout;
}

# ------------------------------------------------------------------

sub psi_tail
{
	my $num_stars = $_[0];
	my $word_count = $_[1];
	my $num_choices = $_[2];
	my $wadjust = $_[3];
	my $chat_goal = "   (Concept \"AIML chat subsystem goal\")\n";
	my $demand = "   (psi-demand \"AIML chat demand\")\n";

	# Stupid hack for rule priority, for lack of something better.
	# Adjust weights so that more than one star is strongly punished.
	# That's in order to supress the pattern "* are *" which matches
	# any sentence with the word "are" in it. Skanky rule, maybe
	# it should be ditched.  More generally, some kind of weighting
	# formula should be developed, one that is more "scientifically"
	# motivated.  Of course, this breaks the AIML spec, which does
	# not use randomness or weighting in it; however, we want to avoid
	# strict determinism here, as its not very realistic.  Note that
	# this kind of randomness will break the use of opencog aiml for
	# any sort of customer-support system that insists on exactly a
	# given fixed answer to a given situation.  Oh well; that's not
	# what we're after, here.
	#
	# The $kill= &exp() formula is attempting to kill the probability
	# of very short star-matches, e.g. so that "YOU ARE *" is strongly
	# prefered to "YOU *".  The formula below yeilds:
	# YOU *       $word_count=1 $num_stars=1 $weight= 6.81e-6
	# YOU ARE *   $word_count=2 $num_stars=1 $weight= 8.39e-5
	my $kill= (0.5 + $word_count) * 0.1 * exp(2.0 * $num_stars * ($word_count - 6.0));
	if (1.0 < $kill) { $kill = 1.0; }
	my $weight = $base_priority * $kill;
	$weight = $weight / $num_choices;

	# Adjust the weight by the desired adjustment.
	$weight *= $wadjust;

	# my $goal_truth = "   (stv 1 0.8)\n";
	my $goal_truth = "   (stv 1 $weight)\n";
	my $rule_tail = $chat_goal . $goal_truth . $demand;

	$rule_tail;
}

# If there is a weight file, and the pattern an be found in
# the weight file, then get that weight.
sub get_weight
{
	my $cattext = $_[0];
	if ($cattext =~/<pattern>(.*)<\/pattern>\s*<topic>(.*)<\/topic>\s*<that>(.*)<\/that>/)
	{
		my $pat = $1;
		my $topic = $2;
		my $that = $3;

		my $key = make_wkey($pat, $that, $topic);
		if (defined $weights{$key})
		{
			my $logli = $weights{$key};

			# XXX FIXME -- this modulation is totally bogus,
			# as it results in values that will always be 0.9999 pretty
			# much no matter what.  So some other formula has to be used.
			# 2 July 2016 - sent email asking about this.
			# my $prob = 1.0 - exp(-$logli);

			# Utter and pure hack: the largest entropies in the file are
			# about 12 or 13. So use that as a scale max, and invert the
			# direction.
			my $prob = $logli / 12.0;
			if ($prob > 1.0) { $prob = 1.0; }
			return $prob;
		}
	}
	1.0;
}

# ------------------------------------------------------------------
# Second pass

open (FIN,"<$intermediateFile");
my $curPath="";
my %overwriteSpace=();
my $psi_ctxt = "";
my $psi_goal = "";

my $have_raw_code = 0;
my $curr_raw_code = "";

my $cattext = "";

my $rule_count = 0;
my $file_count = 1;

if ($outDir ne '')
{
	mkdir $outDir;
	open (FOUT,">" . $outDir . "/aiml-" . $file_count . ".scm");
}
else
{
	open (FOUT,">" . $outFile);
}

my $date = localtime();
print FOUT ";;\n;; Generated by aiml2psi.pl version $ver on $date\n;;\n";
print FOUT ";; AIML Source directory = $aimlDir\n;;\n";
print FOUT ";; Command line was\n;;\n";
print FOUT ";;      $cmdline\n;;\n";

while (my $line = <FIN>)
{
	chomp($line);
	if (length($line) < 1) { next; }
	my @parms = split(/\,/, $line);
	my $cmd = $parms[0] || "";

	# To accept a pattern like this "<pattern>0</pattern>" as well
	my $arg = ($parms[1] || $parms[1] eq 0)? $parms[1] : "";

	if (length($cmd) < 1) { next; }

	# Un-do the comma-damage up above.
	$arg =~ s/#Comma/,/g;

	# esacpe quote marks.
	$arg =~ s/"/\\"/g;

	# Undo html markup
	$arg =~ s/&lt;/</g;

	# CATEGORY
	if ($cmd eq "CATBEGIN")
	{
		$psi_ctxt .= "   (list (AndLink\n";
	}
	if ($cmd eq "CATTEXT")
	{
		$cattext = $line;
		$cattext =~ s/^CATTEXT,//g;
		$cattext =~ s/\#Comma/,/g;

		# Undo html markup
		$cattext =~ s/&lt;/</g;

		# Unescape escaped single-quotes.
		$cattext =~ s/\\'/'/g;

		# Escape back-slashes
		$cattext =~ s/\\/\\\\/g;

		# Escape double-quotes.
		$cattext =~ s/"/\\"/g;

		# Trim leading and trailing whitespace
		$cattext =~ s/^\s*//g;
		$cattext =~ s/\s*$//g;

	}
	if ($cmd eq "PATH")
	{
		$curPath = $arg;
		# $psi_ctxt .= "; PATH --> $curPath\n";
	}

	if ($cmd eq "CATEND")
	{
		my $rule = "";
		# Number of stars is one less than the current index.
		my $num_stars = $star_index - 1;

		if ($have_raw_code)
		{
			# Random sections are handled by duplicating
			# the rule repeatedly, each time with the same
			# premise template, but each with a diffrerent output.
			if ($curr_raw_code =~ /(.*?)<random>(.*?)<\/random>(.*)/)
			{
				$rule .= &process_random($curr_raw_code, $cattext,
							$num_stars, $psi_ctxt);
			}
			else
			{
				my $wadj = &get_weight($cattext);
				$rule = ";;; COMPLEX CODE BRANCH\n";
				$rule .= ";;; " . $cattext . "\n";
				$rule .= "(psi-rule\n";
				$rule .= "   ;; context\n";
				$rule .= $psi_ctxt;
				$rule .= "   ;; action\n";
				$rule .= "   (ListLink\n";
				$rule .= &process_category("      ", $curr_raw_code);
				$rule .= "   )\n";
				$rule .= &psi_tail($num_stars, $pat_word_count, 1, $wadj);
				$rule .= ")\n";
			}
			$have_raw_code = 0;
		}
		else
		{
			my $wadj = &get_weight($cattext);
			$rule = ";;; NO RAW CODE\n";
			$rule .= ";;; $cattext\n";
			$rule .= "(psi-rule\n";
			$rule .= "   ;; context\n";
			$rule .= $psi_ctxt;
			$rule .= "   ;; action\n";
			$rule .= $psi_goal;
			$rule .= &psi_tail($num_stars, $pat_word_count, 1, $wadj);
			$rule .= ") ; CATEND\n";     # close category section

			$psi_goal = "";
		}

		if ($overwrite)
		{
			# Overwrite in a hash space indexed by the current path.
			$overwriteSpace{$curPath} = $rule;
		}
		else
		{
			# Not merging, so just write it out.
			print FOUT "$rule\n";

			# OK, so the current guile compiler is broken, it appears
			# to have a runtime of N^2 where N is the size of the file.
			# Avoid an excessively long compile time by writing lots of
			# small files.  At this time (June 2016, guile-2.0), really
			# tiny files work best.
			$rule_count ++;
			if ($outDir ne '' and $rule_count > 40)
			{
				$rule_count = 0;
				$file_count ++;

				print FOUT "; ---------- end of file ----------\n";
				print FOUT "*unspecified*\n";
				close (FOUT);
				open (FOUT,">" . $outDir . "/aiml-" . $file_count . ".scm");
			}
		}
		$psi_ctxt = "";
		# $psi_goal = "";
	}

	# We are going to have to fix this for the various stars and
	# variables, but it is a start.

	# PATTERN
	if ($cmd eq "PAT")
	{
		my $curr_pattern = $arg;
		$star_index = 1;
		$word_count = 0;
		$psi_ctxt .= &print_predicate_tag("pattern", "      ", lc $curr_pattern);
		$pat_word_count = $word_count;
	}

	#TOPIC
	if ($cmd eq "TOPIC")
	{
		if ($arg ne "" and $arg ne "*") {
			my $curr_topic = $arg;
			$psi_ctxt .= "      ; Context with topic!\n";
			$psi_ctxt .= &print_predicate_tag("topic", "      ", lc $curr_topic);
		}
	}

	# THAT
	if ($cmd eq "THAT")
	{
		if ($arg ne "" and $arg ne "*") {
			my $curr_that = $arg;
			$psi_ctxt .= "      ; Context with that!\n";
			$psi_ctxt .= &print_predicate_tag("that", "      ", lc $curr_that);
		}
	}

	#template
	if ($cmd eq "TEMPLATECODE")
	{
		$psi_ctxt .= "   )) ;TEMPLATECODE\n";  # close pattern section

		$arg =~ s/\"/\'/g;

		$have_raw_code = 1;
		$curr_raw_code = $arg;
	}

	if ($cmd eq "TEMPATOMIC")
	{
		$psi_ctxt .= "   )) ;TEMPATOMIC\n";  # close pattern section
		# The AIML code was just a list of words, so just set up for a
		#word sequence.
		$psi_goal = "   (ListLink\n";
	}

	if ($cmd eq "TEMPWRD")
	{
		$arg = &trim_punct($arg);
		if ($arg ne "")
		{
			# Just another word in the reply chain.
			$psi_goal .= "      " . $wordnode . "\"$arg\")\n";
		}
	}
	if ($cmd eq "TEMPATOMICEND")
	{
		# Just another word in the reply chain.
		$psi_goal .= "   ) ; TEMPATOMICEND\n";
	}
}

# If merging, then sort and write out.
if ($overwrite)
{
	foreach my $p (sort keys %overwriteSpace)
	{
		print FOUT "$overwriteSpace{$p}\n";
	}
}

print FOUT "; ---------- end of file ----------\n";
print FOUT "*unspecified*\n";

close(FIN);
close(FOUT);

print "Processed $rule_count rules\n";
exit;
=for comment

original AIML :

<category>
 <pattern>Hello</pattern>
 <template> Hi there. </template>
</category>

has implied fields of <topic>*</topic>  and <that>*</that>:

<category>
 <pattern>Hello</pattern>
 <topic>*</topic>
 <that>*</that>
 <template> Hi there. </template>
</category>

which is translates to an intermediate sequence of

CATBEGIN,0
PAT,Hello
PWRD,Hello
PATEND,0
TOPIC,*
TOPICSTAR,1
TOPICEND,0
THAT,*
THATSTAR,1
THATEND,0
PATH,<input>/Hello/<topic>/*/<that>/*
TEMPLATE, Hi there.
CATTEXT, <category> <pattern>Hello</pattern> <topic>*</topic> <that>*</that> <template> Hi there. </template> </category>
CATEND,0


=OpenCog equivalents
* R1 example.
```
; Every DefinedSchema must have a globally unique name, and the
; original category text is as good a name as any.  Useful for
; debugging.  In all other respects, the actual name chosen does
; not matter. The MemberLink simply describes the DefinedSchema
; as belonging to a particular rulebase; in this case, the rulbase
; is called (Concept "*-AIML-rulebase-*").  The actual name of
; the rulebase does not matter.
(MemberLink
   (DefinedSchema "<category> <pattern>Hello</pattern> <topic>*</topic> <that>*</that> <template> Hi there. </template> </category>")
   (Concept "*-AIML-rulebase-*"))

; This actually defines the schema. Note that the name used here must be
; *identical* to that above.  The Implication has two parts: it has a
; simple if-then form.  Note that a very simple implication is used:
; a BindLink is NOT used!  This is for a very important reason: it
; isolates the rule from the actual form that is used to represent
; sentences in the atomspace.  It is straight-forward to convert the
; ImplicationLinks into actual patterns that can match the current
; input text.  The AIML importer does NOT need to make any assumptions
; about what the cirrent sentence representation is.
(DefineLink
   (DefinedSchema "<category> <pattern>Hello</pattern> <topic>*</topic> <that>*</that> <template> Hi there. </template> </category>")
   (Implication
      (And
         (ListLink
            (Word "hello")
         )
      )
      (ListLink
         (TextNode "Hi there.")
      )
   )
)
```

Another example: a simple SRAI:
```
; Notice the general similarity to the above.  The SRAI tag is
; converted to a DefinedSchema, whose execution is tiggered
; whenever the rule is run.  Notice also the handling of the
; star with a multi-word GlobNode.
;
; This perl script correctly handles nested SRAI.  It also handles
; random-choice responses: it splits these into multiple rules, so
; that they can be more easily merged with other stimulous and chat
; sources.
;
; Other AIML tags are also converted into DefinedSchema; e.g. the
; <person> tage is converted into (DefinedSchema "AIML-tag person").
;
(MemberLink
   (DefinedSchema "<category>    <pattern>SORRY *</pattern> <template><srai>sorry</srai></template> </category>")
   (Concept "*-AIML-rulebase-*"))
(DefineLink
   (DefinedSchema "<category>    <pattern>SORRY *</pattern> <template><srai>sorry</srai></template> </category>")
   (Implication
      (And
         (ListLink
            (Word "sorry")
            (Glob "$star-1")
         )
      )
      (ListLink
         (ExecutionOutput
            (DefinedSchema "AIML-tag srai")
            (ListLink
               (Text "sorry")
            ))
      )
   )
)
```


=end comment
