#! /usr/bin/env perl
#
# Convert AIML files to OpenCog Atomese.
#
# The perl script converts AIML XML into OpenCog Atomese.  See the
# bottom for an example of the output format, and a breif discussion
# about the design choices taken.  This script "works", in that it
# generates valid Atomese that can actually be imported into the
# atomspace.
#
# As of April 2016, the idea of importing AIML is mothballed: so,
# although the conversion and import works, the surrounding code
# to attach the AIML rules into the rest of the OpenCog chat
# infrastructure has not been created, and probably wont be.  The
# reason for this is that there is no compelling AIML content that
# is in any way useful to the current plans for OpenCog.  I think
# we've moved past AIML in terms of what we can accomplish.
#
# Copyright (c) Kino Coursey 2015
# Copyright (c) Linas Vepstas 2016
#
use Getopt::Long qw(GetOptions);
use strict;

my $ver = "0.2.0";
my $debug;
my $help;
my $version;
my $overwrite;
my $aimlDir ='.';
my $intermediateFile = 'aiml-flat.txt';
my $outDir = '';
my $outFile = 'aiml-rules.scm';

GetOptions(
    'dir=s' => \$aimlDir,
    'debug' => \$debug,
    'help' => \$help,
    'last-only' => \$overwrite,
    'version' => \$version,
    'intermediate=s' => \$intermediateFile,
    'out=s' => \$outDir,
) or die "Usage: $0 [--debug] [--help] [--version] [--last-only] [--dir <AIML source directory>] [--intermediate <IMMFile>] [--out <output directory>]\n";

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
	print "   --out <directory>       Directory for OpenCog output files\n";
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

my $wordnode = "(Word ";
# my $wordnode = "(Concept ";

my $psi_goal = "   (Concept \"AIML chat goal\")\n";
my	$goal_truth = "   (stv 1 0.8)\n";
my $demand = "   (psi-demand \"AIML chat\" 0.97)\n";
my $psi_tail = $psi_goal . $goal_truth . $demand;

# split_string -- split a string of words into distinct nodes.
sub split_string
{
	my $indent = $_[0];
	my $text = $_[1];
	my @words = split(/ /, $text);
	my $tout = "";
	for my $wrd (@words)
	{
		if ($wrd ne "")
		{
			$tout .= $indent . $wordnode . "\"$wrd\")\n";
		}
	}
	$tout;
}

# process_star -- deal with the star XML
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

# process_multi_star -- multiple star extraction
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_multi_star
{
	my $indent = $_[0];
	my $text = $_[1];

	my $tout = "";
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
				$tout .= &split_string($indent, $t);
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
				$tout .= &split_string($indent, $t);
			}
		}
		elsif ($star ne "")
		{
			# At this point, we expect just a string of words.
			$tout .= &split_string($indent, $star);
		}
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

	# FIXME, should be like the star loop, above.
	$tout .= &process_aiml_tags($indent, $1);
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag $tag\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (ListLink\n";
	$tout .= &process_aiml_tags($indent . "         ", $2);
	$tout .= $indent . "   )))\n";
	if ($3 ne "")
	{
		$tout .= &split_string($indent, $3);
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

	$text =~ /(.*?)<set name='(.*?)'>(.*)<\/set>(.*)/;

	# FIXME, should be like the star loop, above.
	$tout .= &split_string($indent, $1);
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag set\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "      (Concept \"" . $2 . "\")\n";
	$tout .= $indent . "      (ListLink\n";
	$tout .= &process_aiml_tags($indent . "         ", $3);
	$tout .= $indent . "   )))\n";
	if ($4 ne "")
	{
		$tout .= &split_string($indent, $4);
	}
	$tout;
}

# process_named_tag -- process a generic tag that has a name
#
# First argument: the tag name
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_named_tag
{
	my $tag = $_[0];
	my $indent = $_[1];
	my $text = $_[2];
	my $tout = "";

	# Multiple gets may appear in one reply.
	$text =~ /<$tag name=/;

	my @gets = split /<$tag/, $text;
	foreach my $get (@gets)
	{
		if ($get =~ /name='(.*)'\/>(.*)/)
		{
			$tout .= $indent . "(ExecutionOutput\n";
			$tout .= $indent . "   (DefinedSchema \"AIML-tag $tag\")\n";
			$tout .= $indent . "   (ListLink\n";
			$tout .= $indent . "      (Concept \"$1\")\n";
			$tout .= $indent . "   ))\n";
			$tout .= &split_string($indent, $2);
		}
		else
		{
			$tout .= &split_string($indent, $get);
		}
	}
}

# process_that -- process a that tag
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_that
{
	my $indent = $_[0];
	my $text = $_[1];
	my $tout = "";

	$text =~ /(.*?)<that\/>(.*)/;

	# FIXME, should be like the star loop, above.
	$tout .= &split_string($indent, $1);
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag that\")\n";
	$tout .= $indent . "   (ListLink))\n";
	if ($2 ne "")
	{
		$tout .= &process_aiml_tags($indent, $2);
	}
	$tout;
}

# process_input -- process a input tag
#
# First argument: white-space indentation to insert on each line.
# Second argument: the actual text to unpack
sub process_input
{
	my $indent = $_[0];
	my $text = $_[1];
	my $tout = "";

	$text =~ /<input\s*index\s*=\s*'(\d+)'\s*\/>/;
	$tout .= $indent . "(ExecutionOutput\n";
	$tout .= $indent . "   (DefinedSchema \"AIML-tag input\")\n";
	$tout .= $indent . "   (ListLink\n";
	$tout .= $indent . "       (Number \"$1\")))\n";
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

	# XXX FIXME ? This is supposed to be equivalent to
	# <person><star/></person> however, in the actual AIML texts,
	# there is no actual star, so its broken/invalid sytax.
	$text =~ s/<person\/>/<person><star\/><\/person>/g;

	# Convert mangled commas, from pass 1
	$text =~ s/#Comma/,/g;

	# Unescape escaped single-quotes.
	$text =~ s/\\'/'/g;

	# Escape back-slashes
	$text =~ s/\\/\\\\/g;

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

	my $tout = "";

	# Find the very first angle bracket
	if ($text =~ /(.*?)<(.*)/)
	{
		my $tag = $2;
		if ($tag =~ /^srai>/)
		{
			$tout .= process_tag("srai", $indent, $text);
		}
		elsif ($tag =~ /^star/)
		{
			$tout .= &process_multi_star($indent, $text);
		}
		elsif ($tag =~ /^think>/)
		{
			$tout .= process_tag("think", $indent, $text);
		}
		elsif ($tag =~ /^set name/)
		{
			$tout .= process_set($indent, $text);
		}
		elsif ($tag =~ /^person>/)
		{
			$tout .= process_tag("person", $indent, $text);
		}
		elsif ($tag =~ /^that\/>/)
		{
			$tout .= process_that($indent, $text);
		}
		elsif ($tag =~ /^input/)
		{
			$tout .= process_input($indent, $text);
		}
		elsif ($tag =~ /^get name/)
		{
			$tout .= process_named_tag("get", $indent, $text);
		}
		elsif ($tag =~ /^bot name/)
		{
			$tout .= process_named_tag("bot", $indent, $text);
		}
		elsif ($text =~ /<!--.*-->/)
		{
			# WTF is <!-- REDUCTION --> ??? whatever it is we don't print it.
			$tout .= $indent . "; WTF $text\n";
		}
	}
	else
	{
		$tout .= &split_string($indent, $text);
	}
	$tout;
}
# ------------------------------------------------------------------
# Second pass

open (FIN,"<$intermediateFile");
my $curPath="";
my %overwriteSpace=();
my $psi_ctxt = "";
my $psi_goal = "";

my $have_topic = 0;
my $curr_topic = "";

my $have_that = 0;
my $curr_that = "";

my $have_raw_code = 0;
my $curr_raw_code = "";

my $cattext = "";

my $star_index = 1;

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

	# Undo html markup
	$arg =~ s/&lt;/</g;

	# CATEGORY
	if ($cmd eq "CATBEGIN")
	{
		$psi_ctxt .= "   (list\n";
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

					$rule .= ";;; random choice $i of $nc: ";
					$rule .= $cattext . "\n";

					$rule .= "(psi-rule\n";
					$rule .= "   ; context\n";
					$rule .= $psi_ctxt;
					$rule .= "   ; action\n";
					$rule .= "   (ListLink\n";
					$rule .= &process_category("      ", $ch);
					$rule .= "   )\n";
					$rule .= $psi_tail;
					$rule .= ") ; random choice $i of $nc\n\n";  # close category section
					$i = $i + 1;
				}
         }
			else
			{
				$rule = ";;; COMPLEX CODE BRANCH\n";
				$rule .= ";;; " . $cattext . "\n";
				$rule .= "(psi-rule\n";
				$rule .= "   ;; context\n";
				$rule .= $psi_ctxt;
				$rule .= "   ;; action\n";
				$rule .= $psi_goal;
				$rule .= "   (ListLink\n";
				$rule .= &process_category("      ", $curr_raw_code);
				$rule .= "   )\n";
				$rule .= $psi_tail;
				$rule .= ")\n";
			}
			$have_raw_code = 0;
		}
		else
		{
			$rule = ";;; NO RAW CODE\n";
			$rule .= ";;; $cattext\n";
			$rule .= "(psi-rule\n";
			$rule .= "   ;; context\n";
			$rule .= $psi_ctxt;
			$rule .= "   ;; action\n";
			$rule .= $psi_goal;
			$rule .= $psi_tail;
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
		$star_index = 0;
		$psi_ctxt .= "      (ListLink\n";
	}
	if ($cmd eq "PWRD")
	{
		# Use lower-case ...
		$arg = lc $arg;
		$psi_ctxt .= "         " . $wordnode . "\"$arg\")\n";
	}
	if ($cmd eq "PSTAR")
	{
		$star_index = $star_index + 1;
		$psi_ctxt .= "         (Glob \"\$star-$star_index\")\n";
	}
	if ($cmd eq "PUSTAR")
	{
		$psi_ctxt .= "         (GlobbyBlobbyNode \"_\")\n";
	}
	if ($cmd eq "PBOTVAR")
	{
		$psi_ctxt .= "         (BOTVARNode \"$arg\")\n";
	}
	if ($cmd eq "PSET")
	{
		$psi_ctxt .= "         (XConceptxxNode \"$arg\") ; Huh?\n";
	}
	if ($cmd eq "PATEND")
	{
		$psi_ctxt .= "      ) ; PATEND\n";
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
		$psi_ctxt .= "; TOPICBOTVAR $arg\n";
	}
	if ($cmd eq "TOPICSET")
	{
		$have_topic = 1;
		$curr_topic = $arg;
		$psi_ctxt .= "; TOPICSET $arg\n";
	}
	if ($cmd eq "TOPICEND")
	{
		if ($have_topic)
		{
# XXX this is wrong -- fixme ...
			#$psi_ctxt .= "         (State\n";
			#$psi_ctxt .= "            (Anchor \"\#topic\")\n";
			#$psi_ctxt .= "            (Concept \"$curr_topic\")\n";
			#$psi_ctxt .= "         )\n";
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
		$psi_ctxt .= "; THATBOTVAR $arg\n";
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
			$psi_ctxt .= "         (State\n";
			$psi_ctxt .= "            (Anchor \"\#that\")\n";
			$psi_ctxt .= "            (Concept \"$curr_that\")\n";
			$psi_ctxt .= "         )\n";
		}
		$have_that = 0;
	}

	#template
	if ($cmd eq "TEMPLATECODE")
	{
		$psi_ctxt .= "   ) ;TEMPLATECODE\n";  # close pattern section

		$arg =~ s/\"/\'/g;

		$have_raw_code = 1;
		$curr_raw_code = $arg;
	}

	if ($cmd eq "TEMPATOMIC")
	{
		$psi_ctxt .= "   ) ;TEMPATOMIC\n";  # close pattern section
		# The AIML code was just a list of words, so just set up for a
		#word sequence.
		$psi_goal = "   (ListLink\n";
	}

	if ($cmd eq "TEMPWRD")
	{
		# Unescape escaped single-quotes.
		$arg =~ s/\\'/'/g;

		# Escape back-slashes
		$arg =~ s/\\/\\\\/g;

		# Escape double-quotes.
		$arg =~ s/"/\\"/g;

		# Just another word in the reply chain.
		$psi_goal .= "      " . $wordnode . "\"$arg\")\n";
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
