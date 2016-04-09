#! /usr/bin/env perl
#
# Convert AIML files to OpenCog Atomese.
#
# Copyright (c) Kino Coursey 2015
# Copyright (c) Linas Vepstas 2016
#
use Getopt::Long qw(GetOptions);
use strict;

my $ver = "0.1.0";
my $debug;
my $help;
my $version;
my $overwrite;
my $aimlDir ='.';
my $intermediateFile = 'flat-aiml.txt';
my $finalFile = 'aiml.scm';
my $rulebase = "*-AIML-rulebase-*";

GetOptions(
    'dir=s' => \$aimlDir,
    'debug' => \$debug,
    'help' => \$help,
    'rulebase-name=s' => \$rulebase,
    'last-only' => \$overwrite,
    'version' => \$version,
    'intermediate=s' => \$intermediateFile,
    'out=s' => \$finalFile,
) or die "Usage: $0 [--debug] [--help] [--version] [--rulebase-name] [--last-only] [--dir
<AIML source directory>] [--intermediate <IMMFile>] [--out <OpenCog file>]\n";

if ($help)
{
	print "Convert AIML markup files to OpenCog Atomese files.\n";
	print "\n";
	print "Usage: $0 [--debug] [--help] [--version] [--rulebase-name] [--last-only] [--dir <AIML source directory>] [--intermediate <IMMFile>] [--out <OpenCog file>]\n";
	print "   --debug                 Enable debugging (if any).\n";
	print "   --help                  Print these helpful comments.\n";
	print "   --version               Print version, current version '$ver'\n";
	print "   --rulebase-name         Specify a name for the ruleset, default '$rulebase'\n";
	print "   --last-only             Only the last category is output.\n";
	print "   --dir <directory>       AIML source directory, default: '$aimlDir'\n";
	print "   --intermediate <file>   Intermediate file, default: '$intermediateFile'\n";
	print "   --out <file>          OpenCog output file, default is '$finalFile'\n";
	die "\n";
}

if ($version)
{
	print "version $ver\n";
	die "\n";
}


#$src = 'core65.aiml';

# Conversion is done in a two-pass process.  The first pass flattens
# the AIML format into a simplified linear format.  A second pass
# converts this flattened format into Atomese.

print "\n AIML Source directory = $aimlDir\n";
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
				my $tpat = "\<\/pattern\> \<topic\>". $topicx ."\<\/topic\> \<that\>";
				$c =~ s/\<\/pattern\> \<that\>/$tpat/;
			}
			my @pat = $c =~ m/\<pattern\>(.*?)\<\/pattern\>/;
			my @top = $c =~ m/\<topic\>(.*?)\<\/topic\>/;
			my @that  = $c =~ m/\<that\>(.*?)\<\/that\>/;
			my @template  = $c =~ m/\<template\>(.*?)\<\/template\>/;
			if( @pat == 0) {next;}
			if( @template == 0) {next;}
			if (@that == 0) { push(@that,"");}
			if (@top == 0) { push(@top,"");}

			# Special cases.
			#	pattern side <set>{NAME}</set> and <bot name=""/>
			#
			if (@pat >0) {$pat[0]=~ s/\<bot name/\<bot_name/gi; }
			if (@pat >0) {$pat[0]=~ s/\<set> /<set>/gi; }
			if (@top >0) {$top[0]=~ s/\<set> /<set>/gi; }
			if (@that >0) {$that[0]=~ s/\<set> /<set>/gi; }#

			if (@pat >0)  {$pat[0]=~ s/ <\/set>/<\/set>/gi; }
			if (@top >0)  {$top[0]=~ s/ <\/set>/<\/set>/gi; }
			if (@that >0) {$that[0]=~ s/ <\/set>/<\/set>/gi; }

			my @PWRDS = split(/ /,$pat[0]);
			my @TWRDS = split(/ /,$that[0]);
			my @TPWRDS = split(/ /,$top[0]); #
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
					print FOUT "TEMPATOMIC,0\n";
					my @TEMPWRDS = split(/ /,$template[0]); #
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

# What node should we use for output text ???
# my $textnode = "(TextNode ";
my $textnode = "(Concept ";

# Handle expressions like <star/> and <star index='2'/> and so on.
sub process_star
{
	my $text = $_[0];
	my $tout = "";

	if ($text =~ /<star\s*\/>/)
	{
		$tout .= "(Glob \"\$star-1\")";
	}
	elsif ($text =~ /<star\s*index\s*=\s*'(\d+)'\s*\/>/)
	{
		$tout .= "(Glob \"\$star-$1\")";
	}
	else
	{
		# Error .. should throw here I guess
		$tout .= "(AIEEEE! \"$text\")";
	}

	$tout;
}

sub process_aiml_tags;

# Decode and print nested SRAI tags.  This turns out to be a complicted
# pain. The problem with srai is that it can be nested, and its hard to
# use regexes to walk that nesting in a reliable way. (I don't feel like
# debugging a recursive regex.) So this is implemented as a brute-force
# string search, where I just count the nesting levels by hand. Even so,
# this is painfully verbose.
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_srai
{
	my $indent = $_[0];
	my $text = $_[1];

	my $tout = "";

	my $nest = 0;

	# Split on all XML tags.
	my @tags = split /</, $text;
	my $contig = shift @tags;

	# Iterate over all the tags.
	for my $tag (@tags)
	{

		if ($tag =~ /^srai>(.*)/)
		{
			my $srt = $1;

			# Found an srai tag. If the tag nesting level is zero,
			# then process everything that came before it, (if there
			# is anything)
			if ($nest == 0)
			{
				$contig =~ s/^\s*//;
				$contig =~ s/\s*$//;
				if ($contig ne "")
				{
					$tout .= &process_aiml_tags($indent, $contig);
				}
				$contig = $srt;  # strip off the starting tag.
			}
			else
			{
				$contig .= "<" . $tag;
			}
			$nest = $nest + 1;
		}
		elsif ($tag =~ /^\/srai>(.*)/)
		{
			my $post = $1;

			# Found an srai end-tag. If the nest level is 1, then
			# print out what we've got; else accumulate some more.
			if ($nest == 1)
			{
				$contig =~ s/^\s*//;
				$contig =~ s/\s*$//;
				$tout .= $indent . "(ExecutionOutput\n";
				$tout .= $indent . "   (DefinedSchema \"AIML-tag srai\")\n";
				$tout .= $indent . "   (ListLink\n";
				$tout .= &process_aiml_tags("      " . $indent, $contig);
				$tout .= $indent . "   ))\n";

				$contig = $post;
			}
			else
			{
				$contig .= "<" . $tag;
			}
			$nest = $nest - 1;
		}
		else
		{
			$contig .= "<" . $tag;
		}
	}

	$contig =~ s/^\s*//;
	$contig =~ s/\s*$//;
	if ($contig ne "")
	{
		$tout .= &process_aiml_tags($indent, $contig);
	}
	$tout;
}

# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_aiml_tags
{
	my $indent = $_[0];
	my $text = $_[1];

	# Expand defintion of <sr/>
	$text =~ s/<sr\/>/<srai><star\/><\/srai>/g;

	$text =~ s/#Comma/,/g;
	$text =~ s/^\s*//;
	$text =~ s/\s*$//;

	my $tout = "";

	# <srai>'s can be nested, one inside the other.
	# Maybe this could be solved with a recursive regex, but
	# I don't feel like debugging that.  So instead, this
	# uses a brute-force approach.
	if ($text =~ /<srai>/)
	{
		$text = lc $text;
		$tout .= &process_srai($indent, $text);
	}

	elsif ($text =~ /(.*)<person>(.*)<\/person>(.*)/)
	{
		# FIXME, should be like the loop, below.
		$tout .= $indent . $textnode . "\"$1\")\n";
		$tout .= $indent . "(ExecutionOutput\n";
		$tout .= $indent . "   (DefinedSchema \"AIML-tag person\")\n";
		$tout .= $indent . "   (ListLink\n";
		$tout .= &process_aiml_tags($indent . "      ", $2);
		$tout .= $indent . "   ))\n";
		if ($3 ne "")
		{
			$tout .= $indent . $textnode . "\"$3\")\n";
		}
	}

	elsif ($text =~ /<star/)
	{
		my @stars = split /<star/, $text;
		foreach my $star (@stars)
		{
			$star =~ s/^\s*//;
			$star =~ s/\s*$//;
			if ($star =~ /index='(\d+)'.*\/>(.*)/)
			{
				$tout .= $indent . &process_star("<star index='" . $1 . "'\/>") . "\n";

				my $t = $2;
				$t =~ s/^\s*//;
				$t =~ s/\s*$//;
				if ($t ne "")
				{
					$tout .= $indent . $textnode . "\"$t\")\n";
				}
			}
			elsif ($star =~ /\/>(.*)/)
			{
				$tout .= $indent . &process_star("<star \/>") . "\n";
				my $t = $1;
				$t =~ s/^\s*//;
				$t =~ s/\s*$//;
				if ($t ne "")
				{
					$tout .= $indent . $textnode . "\"$t\")\n";
				}
			}
			elsif ($star ne "")
			{
				$tout .= $indent . $textnode . "\"$star\")\n";
			}
		}
	}

	elsif ($text =~ /<!--.*-->/)
	{
		# WTF is <!-- REDUCTION --> ??? whatever it is we don't print it.
		$tout .= $indent . "; WTF $text\n";
	}
	else
	{
		$tout .= $indent . $textnode . "\"$text\")\n";
	}
	$tout;
}
# ------------------------------------------------------------------
# Second pass

open (FIN,"<$intermediateFile");
open (FOUT,">$finalFile");
my $curPath="";
my %overwriteSpace=();
my $code = "";

my $have_topic = 0;
my $curr_topic = "";

my $have_that = 0;
my $curr_that = "";

my $have_raw_code = 0;
my $curr_raw_code = "";

my $cattext = "";

my $star_index = 1;

while (my $line = <FIN>)
{
	chomp($line);
	if (length($line) < 1) { next; }
	my @parms = split(/\,/, $line);
	my $cmd = $parms[0] || "";
	my $arg = $parms[1] || "";
	if (length($cmd) < 1) { next; }

	# Un-do the comma-damage up above.
	$arg =~ s/#Comma/,/g;

	# esacpe quote marks.
	$arg =~ s/"/\\"/g;

	# CATEGORY
	if ($cmd eq "CATBEGIN")
	{
		$code = "";
		$code .= "   (Implication\n";
		$code .= "      (And\n";
	}
	if ($cmd eq "CATTEXT")
	{
		$cattext = $line;
		$cattext =~ s/^CATTEXT,//g;
		$cattext =~ s/\#Comma/,/g;
		$cattext =~ s/"/\\"/g;
		$cattext =~ s/^\s*//g;
		$cattext =~ s/\s*$//g;
	}
	if ($cmd eq "PATH")
	{
		$curPath = $arg;
		# $code .= "; PATH --> $curPath\n";
	}

	if ($cmd eq "CATEND")
	{
		my $rule = "";

		if ($have_raw_code)
		{
			# Random sections are handled by duplicating
			# the rule repeatedly, each time with the same
			# premise template, but each with a diffrerent output.
			if ($curr_raw_code =~ /<random>(.*)<\/random>/)
			{
				my $choices = $1;
				$choices =~ s/^\s+//;
				my @choicelist = split /<li>/, $choices;
				shift @choicelist;
				my $i = 1;
				my $nc = $#choicelist + 1;
				foreach my $ch (@choicelist)
				{
					$ch =~ s/<\/li>//;
					$ch =~ s/\s+$//;

					my $name = "random choice $i of $nc: ";
					$name .= $cattext;

					$rule .= "(MemberLink\n";
					$rule .= "   (DefinedSchema \"$name\")\n";
					$rule .= "   (Concept \"$rulebase\"))\n";
					$rule .= "(DefineLink\n";
					$rule .= "   (DefinedSchema \"$name\")\n";
					$rule .= $code;
					$rule .= "      (ListLink\n";
					$rule .= &process_aiml_tags("         ", $ch);
					$rule .= "      )\n";
					$rule .= "   )) ; random choice $i of $nc\n\n";  # close category section
					$i = $i + 1;
				}
         }
			else
			{
				$rule  = "(MemberLink\n";
				$rule .= "   (DefinedSchema \"$cattext\")\n";
				$rule .= "   (Concept \"$rulebase\"))\n";
				$rule .= "(DefineLink\n";
				$rule .= "   (DefinedSchema \"$cattext\")\n";
				$rule .= $code;
				$rule .= "      (ListLink\n";
				$rule .= &process_aiml_tags("         ", $curr_raw_code);
				$rule .= "      )\n";
				$rule .= "   )\n)\n";  # close category section
			}
			$have_raw_code = 0;
		}
		else
		{
			$rule  = "(MemberLink\n";
			$rule .= "   (DefinedSchema \"$cattext\")\n";
			$rule .= "   (Concept \"$rulebase\"))\n";
			$rule .= "(DefineLink\n";
			$rule .= "   (DefinedSchema \"$cattext\")\n";
			$rule .= $code;
			$code .= ") ; CATEND\n";     # close category section
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
		}
		$code = "";
	}

	# We are going to have to fix this for the various stars and
	# variables, but it is a start.

	# PATTERN
	if ($cmd eq "PAT")
	{
		$star_index = 0;
		$code .= "         (ListLink\n";
	}
	if ($cmd eq "PWRD")
	{
		# Use lower-case ...
		$arg = lc $arg;
		$code .= "            (Concept \"$arg\")\n";
	}
	if ($cmd eq "PSTAR")
	{
		$star_index = $star_index + 1;
		$code .= "            (Glob \"\$star-$star_index\")\n";
	}
	if ($cmd eq "PUSTAR")
	{
		$code .= "            (WordNode \"_\")\n";
	}
	if ($cmd eq "PBOTVAR")
	{
		$code .= "            (BOTVARNode \"$arg\")\n";
	}
	if ($cmd eq "PSET")
	{
		$code .= "            (ConceptNode \"$arg\") ; Huh?\n";
	}
	if ($cmd eq "PATEND")
	{
		$code .= "         ) ; PATEND\n";
	}

	#TOPIC
	if ($cmd eq "TOPIC")
	{
		$have_topic = 0;
	}
	if ($cmd eq "TOPICWRD")
	{
		$have_topic = 1;
		$curr_topic = $arg;
	}
	if ($cmd eq "TOPICSTAR")
	{
		$have_topic = 0;
	}
	if ($cmd eq "TOPICUSTAR")
	{
		$have_topic = 0;
	}
	if ($cmd eq "TOPICBOTVAR")
	{
		$code .= "; TOPICBOTVAR $arg\n";
	}
	if ($cmd eq "TOPICSET")
	{
		$have_topic = 1;
		$curr_topic = $arg;
		$code .= "; TOPICSET $arg\n";
	}
	if ($cmd eq "TOPICEND")
	{
		if ($have_topic)
		{
			$code .= "         (State\n";
			$code .= "            (Anchor \"\#topic\")\n";
			$code .= "            (Concept \"$curr_topic\")\n";
			$code .= "         )\n";
		}
		$have_topic = 0;
	}

	# THAT
	if ($cmd eq "THAT")
	{
		$have_that = 0;
	}
	if ($cmd eq "THATWRD")
	{
		$have_that = 1;
		$curr_that = $arg;
	}
	if ($cmd eq "THATSTAR")
	{
		$have_that = 0;
	}
	if ($cmd eq "THATUSTAR")
	{
		$have_that = 0;
	}
	if ($cmd eq "THATBOTVAR")
	{
		$code .= "; THATBOTVAR $arg\n";
	}
	if ($cmd eq "THATSET")
	{
		$have_that = 1;
		$curr_that = $arg;
	}
	if ($cmd eq "THATEND")
	{
		if ($have_that)
		{
			$code .= "         (State\n";
			$code .= "            (Anchor \"\#that\")\n";
			$code .= "            (Concept \"$curr_that\")\n";
			$code .= "         )\n";
		}
		$have_that = 0;
	}

	#template
	if ($cmd eq "TEMPLATECODE")
	{
		$code .= "      ) ;TEMPLATECODE\n";  # close pattern section

		$arg =~ s/\"/\'/g;

		$have_raw_code = 1;
		$curr_raw_code = $arg;
	}

	if ($cmd eq "TEMPATOMIC")
	{
		$code .= "       ) ;TEMPATOMIC\n";  # close pattern section
		# The AIML code was just a list of words, so just set up for a
		#word sequence.
		$code .= "       (ListLink\n";
	}
	if ($cmd eq "TEMPWRD")
	{
		# Just another word in the reply chain.
		$code .= "            (Concept \"$arg\")\n";
	}
	if ($cmd eq "TEMPATOMICEND")
	{
		# Just another word in the reply chain.
		$code .= "       ) ; TEMPATOMICEND\n";
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

close(FIN);
close(FOUT);
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
(BindLink
   (AndLink
      (ListLink
         (WordNode "Hello")
         (VariableNode "$eol")
      )
      (StateLink
         (AnchorNode "#topic")
         (WordNode "*")
         (VariableNode "$topic")
      )
      (StateLink
         (AnchorNode "#that")
         (WordNode "*")
         (VariableNode "$that")
      )
    )
    (StateLink
       (AnchorNode "#reply")
       (WordSequenceLink
            (WordNode "Hi")
            (WordNode "there.")
        )
    )
)


=end comment
