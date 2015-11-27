#! /usr/local/bin/perl -w
use Getopt::Long qw(GetOptions);
use strict;

my $ver = "0.000.1a";
my $debug;
my $help;
my $version;
my $overwrite;
my $aimlDir ='.';
my $intermediateFile = 'pout.txt';
my $finalFile = 'cogcode.txt';

GetOptions(
    'dir=s' => \$aimlDir,
    'debug' => \$debug,
    'help' => \$help,
    'overwrite' => \$overwrite,
    'version' => \$version,
    'intermediate=s' => \$intermediateFile,
    'final=s' => \$finalFile,
) or die "Usage: $0 --debug  --help --version --overwrite --dir <AIML source directory> --intermediate <IMMFile> --final <OpenCog file>\n";

if ($help)
{
	print "Usage: $0 --debug  --help --version --overwrite --dir <AIML source directory> --intermediate <IMMFile> --final <OpenCog file>\n";
	print "   --debug                 enable debugging (if any)\n";
	print "   --help                  print these helpful comments\n";
	print "   --version               script version, current version '$ver'\n";
	print "   --overwrite             last-in-only-out processing of categories\n";
	print "   --dir <directory>       AIML source directory, default is '$aimlDir'\n";
	print "   --intermediate <file>   intermediate file, default is '$intermediateFile'\n";
	print "   --final <file>          OpenCog output file, default is '$finalFile'\n";
	die "\n";
}

if ($version)
{
	print "version $ver\n";
	die "\n";
}


#$src = 'core65.aiml';


print "\n AIML Source directory = $aimlDir\n";
opendir(DIR, "$aimlDir");
my @aimlFiles = grep(/\.aiml/,readdir(DIR));
closedir(DIR);

open FOUT, ">$intermediateFile";
foreach my $af (sort @aimlFiles)
{
	my $textfile="";
    my $aimlSrc = "$aimlDir/$af";
	print " \n\n*****  processing $aimlSrc ****\n";
	# read the entire file in as one string
	open FILE, "$aimlSrc" or die "Couldn't open file: $!"; 
	while (<FILE>){
	 $textfile .= $_;
	}
	close FILE;	
	$textfile .="\n";



	# goal read AIML into a linear neutral format while preserving relevant semantic info
	# like the order of pattern side slot filling stars or sets

	my $topicx = "*";




	# normalize file by removing line feeds and excess spaces
	$textfile =~ s/\r\n/ /gi;
	$textfile =~ s/\n/ /gi;
	$textfile =~ s/\r/ /gi;
	$textfile =~ s/ xml\:space=\"preserve\"//gi;
	$textfile =~ s/ xml\:space=\"default\"//gi;

	while ($textfile =~ /  /) { $textfile =~ s/  / /gi;}

	# normalize so every category has a pattern/topic/that/template entries
	$textfile =~ s/\<\/pattern\> \<template\>/\<\/pattern\> \<that\>*\<\/that\> \<template\>/gi;

	#define where to split for analysis
	$textfile =~ s/<category>/\#\#SPLIT \<category\>/gi;
	$textfile =~ s/<\/category>/\<\/category\>\#\#SPLIT /gi;
	$textfile =~ s/<topic /\#\#SPLIT\<topic /gi;
    $textfile =~ s/<\/topic>/\<\/topic\>\#\#SPLIT /gi;
	$textfile =~ s/<aiml/\#\#SPLIT\<aiml/gi;
	$textfile =~ s/<\/aiml>/\<\/aiml\>\#\#SPLIT /gi;

	my @cats = split(/\#\#SPLIT/,$textfile);

	#it should be one category at a time but it could be on high level topics
	foreach my $c (@cats)
	{
	#	print FOUT "$c\n";
		# processing high level topic conditions
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
		
		#processing general categories
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
			
			# special cases
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
			
			#patterns
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
			
			#topics
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
			
			#that
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
			
			#templates
			# use AIMLIF convention of escaping sequences that are not CSV compliant namely ","-> "#Comma "
			if ( @template > 0)
			{
				$template[0] =~ s/\,/\#Comma /gi;
				$template[0] =~ s/^ //gi;
				$template[0] =~ s/ $//gi; #
				print FOUT "PATH,$path\n";
				
				#will probably have to expand this a bit
				# since it requires representing the performative interpretation of XML that AIML assumes
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
				print FOUT "TEMPATOMICEND,0\n";
			}
			else
			{
				print FOUT "TEMPLATECODE,$template[0]\n";
			}
			
			
			
			print FOUT "TEMPLATE,$template[0]\n";
			print FOUT "CATTEXT,$c\n";
			print FOUT "CATEND,0\n";
			print FOUT "\n";
		}

		
	}
}
close(FOUT);
#pass2


open (FIN,"<$intermediateFile");
open (FOUT,">$finalFile");
my $curPath="";
my %overwriteSpace=();
my $code = "";

while(my $line =<FIN>)
{
	chomp($line);
	if (length($line)<1) {next;}
	my @parms=split(/\,/,$line);
	my $cmd=$parms[0] || "";
	my $arg=$parms[1] || "";
	if (length($cmd)<1) {next;}
	
	# CATEGORY
	if ($cmd eq "CATBEGIN")
	{
		$code .= "(PatternLink\n";
		$code .= "   (SequentialAndLink\n";
	}
	if ($cmd eq "PATH")
	{
		 $curPath = $arg;
		 #$code = "";
		 #print "PATH --> $curPath\n";
	}

	if ($cmd eq "CATEND")
	{
	    $code .= ")\n";     # close category section

		if ($overwrite)
		{
			# overwrite in a hash space indexed by the current path
			$overwriteSpace{$curPath} = $code;
		}
		else
		{
		 # not merging so just write it out
		 print FOUT "$code\n";
		}
		 $code = "";
	}
	
	# we are going to have to fix this for the various stars and variables
	# but it is a start
	
	# PATTERN
	if ($cmd eq "PAT")
	{
		$code .= "      (WordSequenceLink\n";
	}
	if ($cmd eq "PWRD")
	{
		$code .= "         (WordNode \"$arg\")\n";
	}
	if ($cmd eq "PSTAR")
	{
		$code .= "         (WordNode \"*\")\n";
	}
	if ($cmd eq "PUSTAR")
	{
		$code .= "         (WordNode \"_\")\n";
	}
	if ($cmd eq "PBOTVAR")
	{
		$code .= "         (BOTVARNode \"$arg\")\n";
	}
	if ($cmd eq "PSET")
	{
		$code .= "         (ConceptNode \"$arg\")\n";
	}
	if ($cmd eq "PATEND")
	{
		$code .= "         (VariableNode \"\$eol\")\n";
		$code .= "      )\n";
	}

	#TOPIC
	if ($cmd eq "TOPIC")
	{
		$code .= "      (ListLink\n";
		$code .= "         (AnchorNode \"\#topic\")\n";
	}
	if ($cmd eq "TOPICWRD")
	{
		$code .= "         (WordNode \"$arg\")\n";
	}
	if ($cmd eq "TOPICSTAR")
	{
		$code .= "         (WordNode \"*\")\n";
	}
	if ($cmd eq "TOPICUSTAR")
	{
		$code .= "         (WordNode \"_\")\n";
	}
	if ($cmd eq "TOPICBOTVAR")
	{
		$code .= "         (BOTVARNode \"$arg\")\n";
	}
	if ($cmd eq "TOPICSET")
	{
		$code .= "         (ConceptNode \"$arg\")\n";
	}
	if ($cmd eq "TOPICEND")
	{
		$code .= "         (VariableNode \"\$topic\")\n";
		$code .= "      )\n";
	}
	
	# THAT
	if ($cmd eq "THAT")
	{
		$code .= "      (ListLink\n";
		$code .= "         (AnchorNode \"\#that\")\n";
	}
	if ($cmd eq "THATWRD")
	{
		$code .= "         (WordNode \"$arg\")\n";
	}
	if ($cmd eq "THATSTAR")
	{
		$code .= "         (WordNode \"*\")\n";
	}
	if ($cmd eq "THATUSTAR")
	{
		$code .= "         (WordNode \"_\")\n";
	}
	if ($cmd eq "THATBOTVAR")
	{
		$code .= "         (BOTVARNode \"$arg\")\n";
	}
	if ($cmd eq "THATSET")
	{
		$code .= "         (ConceptNode \"$arg\")\n";
	}
	if ($cmd eq "THATEND")
	{
		$code .= "         (VariableNode \"\$that\")\n";
		$code .= "      )\n";
	}	
	
	#template
	if ($cmd eq "TEMPLATECODE")
	{
	    $code .= "     )\n";  # close pattern section

		$arg =~ s/\"/\'/g;

		# just raw AIML code
		$code .= "    (PutLink\n";
		$code .= "       (AnchorNode \"\#reply\")\n";
		$code .= "       (AIMLCODENode \"$arg\")\n";
		$code .= "     )\n";
		
	}	
	if ($cmd eq "TEMPATOMIC")
	{
	    $code .= "    )\n";  # close pattern section
		# the AIML code was just a list of words so just setup for a word sequence
		$code .= "    (PutLink\n";
		$code .= "       (AnchorNode \"\#reply\")\n";
		$code .= "       (WordSequenceLink\n";
	}	
	if ($cmd eq "TEMPWRD")
	{
		#just another word in the reply chain
		$code .= "            (WordNode \"$arg\")\n";
	}	
	if ($cmd eq "TEMPATOMICEND")
	{
		#just another word in the reply chain
		$code .= "        )\n";
		$code .= "    )\n";
	}	
	
	
}

#if merging then sort and write out 
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
PatternLink
   SequentailAndLink
      WordSequenceLink
         WordNode "Hello"
         VariableNode "$eol"     # rest of the input line
      ListLink
         AnchoreNode "#that"
         VariableNode "$that"
      ListLink
         AnchorNode "#topic"
         VariableNode "$topic"
      PutLink                    # if the above conditions are satisfied
         AnchorNode "#reply"     # then this PutLink is triggered.
         WordSequenceLink        # This is the reply.
            WordNode "Hi"
            WordNode "there"
```

Or in more scheme-ish format 

(PatternLink
   (SequentialAndLink
      (WordSequenceLink
         (WordNode "Hello")
         (VariableNode "$eol")
      )
      (ListLink
         (AnchorNode "#topic")
         (WordNode "*")
         (VariableNode "$topic")
      )
      (ListLink
         (AnchorNode "#that")
         (WordNode "*")
         (VariableNode "$that")
      )
    )
    (PutLink
       (AnchorNode "#reply")
       (WordSequenceLink
            (WordNode "Hi")
            (WordNode "there.")
        )
    )
)



=end comment

