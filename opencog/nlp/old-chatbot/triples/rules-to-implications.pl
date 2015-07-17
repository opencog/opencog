#! /usr/bin/env perl
#
# rules-to-implications.pl
#
# Convert Relex framing rule files to OpenCog ImplicationLinks
#
# This perl script takes, as input, implication rules, in the IF .. THEN
# format used by the RelEx framing code, and generates equivalent
# OpenCog ImplicationLinks.
#
# To use, do the following:
#
#    cat some-rule-file.txt | ./rules-to-implications.pl "my-rules"
#
# An example of the kind of input that this script understands is:
#
# ; Sentence: "Lisbon is the capital of Portugaul"
# ; var0=Lisbon, var1=capital var2=Portugaul
# # IF %ListLink("# APPLY TRIPLE RULES", $sent)
#       ^ %WordInstanceLink($var0,$sent)  ; $var0 and $var1 must be
#       ^ %WordInstanceLink($var1,$sent)  ; in the same sentence
#       ^ _subj(be,$var0) ^ _obj(be,$var1)
#       ^ $prep($var1,$var2)              ; preposition 
#       ^ %LemmaLink($var1,$word1)        ; word of word instance
#       ^ $phrase($word1, $prep)          ; convert to phrase
#       THEN ^3_$phrase($var2, $var0) 
#
# Here's another example, showing the use of the ! for NOT:
#
# ; 6
# ; Sentence "Berlin is a city"
# ; var1=Berlin var2=city
# ; Must be careful not to make the pattern too simple, as, for example,
# ; "the captial of Germany is Berlin", the prep is crucial to reversing
# ; the order of the subject and object! Alternately, demand that $var2
# ; is indefinite.
# # IF %ListLink("# APPLY TRIPLE RULES", $sent)
#       ^ %WordInstanceLink($var1,$sent)  ; $var1 and $var2 must be
#       ^ %WordInstanceLink($var2,$sent)  ; in the same sentence
#       ^ %WordInstanceLink(be,$sent)     ; hyp-flag "be" in same sentence
#       ^ _subj(be, $var1) ^ _obj(be, $var2)
#       ^ !DEFINITE-FLAG($var2)           ; accept "a city" but not "the city"
#       ^ !HYP-FLAG(be)                   ; Disallow "Is Berlin a city?"
#       THEN ^3_isa($var2, $var1) 
#
#
# This perl script implements a very simple parser. Some day in the future,
# it would proably make sense to re-write this in terms of lex & yacc. Maybe.
# In fact, this file has already become unmaintainable, and the syntax
# is unsupportable. New rules should NOT be written in this syntax.
#
# Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
# Created in January, March 2009.
#

use strict;

my %word_map = ();

# --------------------------------------------------------------------
# parse_clause -- Split a clause into it's parts. 
#
# A clause is assumed to be either a triple, such as "blah(foo, bar)"
# or a double, such as "ding ( dong )". The parens are required. The
# whitespace is optional. Its a triple, if it has a comma, else its a
# double. Triples return a list of four items: the triple, plus the
# rest of the line (i.e. trailing characters). Doubles only return a
# list of three things.
#
sub parse_clause
{
	my ($clause) = @_;

	# Ditch the white space, this'll make matching easier.
	# XXXX $clause =~ s/\s*//g;
	# No it won't, since it'll trash quoted string literals.

	# Pull the three parts out of the clause.
	# The pattern matches "blah(ding,dong)"
	$clause =~ m/^\s*([\$\!%\:\&\-\w]+)\s*\(\s*([\$%#\"\-\w\s]+)\s*\,\s*([\$%#\"\-\w\s]+)\s*\)(.*)$/;
	my $pred = $1;
	my $item1 = $2;
	my $item2 = $3;
	my $rest = $4;

	my @parts = ();

	# If the above pattern-match failed, then its a double.
	# Note that, sometimes when the match fails, $1 contains garbage.
	# This seems to be a perl bug. But $2 and $3 will still be empty
	if ($pred eq "" || $item1 eq "" || $item2 eq "")
	{
		# The pattern matches "blah(ding)"
		$clause =~ m/^\s*([\$\&!%\-\w]+)\s*\(\s*([\$%#\-\w]+)\s*\)(.*)$/;
		$pred = $1;
		$item1 = $2;
		$rest = $3;

		# Create the list only is the parse succeeded.
		if ($pred ne "")
		{
			@parts = ($pred, $item1, $rest);
		}
	}
	else
	{
		@parts = ($pred, $item1, $item2, $rest);
	}
	@parts;
}

# --------------------------------------------------------------------
sub get_word_id
{
	my ($item) = @_;
	my $id = $word_map{$item};
	if (!defined $id)
	{
		my $cnt = $word_map{"ccc-cnt-ccc"};
		$cnt += 1;
		$word_map{"ccc-cnt-ccc"} = $cnt;

		$id = "\$word-instance-$cnt";
		$word_map{$item} = $id;
	}
	$id;
}

# --------------------------------------------------------------------
# print a variable or a word instance
# The word-instance is printed as a variable; this is joined
# to a particular word, later on, by a LemmaLink.
#
sub print_var_or_word_instance
{
	my ($item, $indent) = @_;

	if ($item =~ /^\$/)
	{
		print "$indent(VariableNode \"$item\")\n";
	}
	else
	{
		# Dang, not a word node, but some unknown word-instance node.
		# print "$indent(WordNode \"$item1\")\n";
		my $id = get_word_id ($item);
		print "$indent(VariableNode \"$id\")\n";
	}
}

# --------------------------------------------------------------------
# Print a triple-style clause. 
# Expects as input a clause, for example, "_subj(be,$var0)", and 
# some whitespace to indent by. Prints, as output, the standard
# OpenCog EvaluationLink equivalent of the clause.
#
sub print_triple_clause
{
	my ($clause, $indent) = @_;

	# Pull the three parts out of the clause.
	my ($pred, $item1, $item2) = parse_clause($clause);

	# Print a copy of the original clause for reference
	print "$indent;; $clause\n";
	print "$indent(EvaluationLink (stv 1.0 1.0)\n";
	if ($pred =~ /^\$/)
	{
		print "$indent   (VariableNode \"$pred\")\n";
	}
	else
	{
		print "$indent   (DefinedLinguisticRelationshipNode \"$pred\")\n";
	}

	print "$indent   (ListLink\n";

	print_var_or_word_instance($item1, $indent . "      ");
	print_var_or_word_instance($item2, $indent . "      ");

	print "$indent   )\n";
	print "$indent)\n";
}

# --------------------------------------------------------------------
# Handle various special-case macros.
#
sub print_function
{
	my ($clause, $indent) = @_;
	print "$indent;; $clause\n";

	# Pull the two parts out of the clause.
	my ($pred, $item1) = parse_clause($clause);

	# Special-case handling for question-answering.
	# A long list of WH-words
	# XXX add ORLink to handle what,where, how etc.
	if ($pred =~ /\&query_var/)
	{
		print "$indent(InheritanceLink (stv 1.0 1.0)\n";
		print_var_or_word_instance($item1, $indent . "   ");
		print "$indent   (DefinedLinguisticConceptNode \"what\")\n";
		print "$indent)\n";
		return;
	}

	# Special-case handling for question-answering code.
	if ($pred =~ /\&declare_answer/)
	{
		print "$indent(ListLink (stv 1.0 1.0)\n";
		print "$indent   (AnchorNode \"# QUERY SOLUTION\")\n";
		print_var_or_word_instance($item1, $indent . "   ");
		print "$indent)\n";
		return;
	}

	print "Error! Unknown function! $clause\n";
}

# --------------------------------------------------------------------
# Print a double-style clause. 
# Expects as input a clause, for example, "DEFINITE-FLAG(ball)", and 
# some whitespace to indent by. Prints, as output, the standard
# OpenCog InheritanceLink equivalent of the clause.
#
sub print_double_clause
{
	my ($clause, $indent) = @_;

	if ($clause =~ /\&/)
	{
		print_function @_;
		return;
	}

	# Pull the two parts out of the clause.
	my ($pred, $item1) = parse_clause($clause);

	# strip out the -FLAG at the end, and lower-case the whole string
	$pred =~ s/-FLAG//;
	$pred =~ tr/[A-Z]/[a-z]/;

	# Print a copy of the original clause for reference
	print "$indent;; $clause\n";
	print "$indent(InheritanceLink (stv 1.0 1.0)\n";
	print_var_or_word_instance($item1, $indent . "   ");
	print "$indent   (DefinedLinguisticConceptNode \"$pred\")\n";
	print "$indent)\n";
}

# --------------------------------------------------------------------
# Print a clause. 
# Expects as input a clause, for example, "_subj(be,$var0)", and 
# some whitespace to indent by. 
#
sub print_clause
{
	my ($clause, $indent) = @_;

	# Pull out the parts of the clause.
	my @parts = parse_clause($clause);

	if ($#parts == 3)
	{
		print_triple_clause(@_);
	}
	else 
	{
		if ($#parts == 2)
		{
			print_double_clause(@_);
		}
	}
}

# --------------------------------------------------------------------
# print_link  -- print a specifically-named link.
# In some cases, the rules need to make explicit ties of one sort or
# another. For example, sometimes a word instance needs to be tied to
# a specific word, *and* that word must be named. Thus, for example,
# a clause of the form: ^ %LemmaLink($var0,$word0)
# The key identifier here is the leading percent sign, which says:
# "interpret what follows literally".
sub print_link
{
	my ($clause, $indent) = @_;

	# Pull the three parts out of the clause.
	$clause =~ s/\s*^\%//;
	my ($link, $item1, $item2) = parse_clause($clause);

	# Print a copy of the original clause for reference
	print "$indent;; $clause\n";
	print "$indent($link (stv 1 1)\n";
	if ($item1 =~ /^\$/)
	{
		print "$indent   (VariableNode \"$item1\")\n";
	}
	elsif ($item1 =~ /^\"#/) # if string starts with hash  mark
	{
		print "$indent   (AnchorNode $item1)\n";
	}
	elsif ($item1 =~ /^\"/) # else a string in general
	{
		print "$indent   (ConceptNode $item1)\n";
	}
	else
	{
		print_var_or_word_instance($item1, $indent . "   ");
	}

	if ($item2 =~ /^\$/)
	{
		print "$indent   (VariableNode \"$item2\")\n";
	}
	elsif ($item2 =~ /^\"#/) # if string starts with hash  mark
	{
		print "$indent   (AnchorNode $item2)\n";
	}
	elsif ($item2 =~ /^\"/) # else a string in general
	{
		print "$indent   (ConceptNode $item2)\n";
	}
	else
	{
		print_var_or_word_instance($item2, $indent . "   ");
	}
	print "$indent)\n";
}


# --------------------------------------------------------------------
# print_exec  -- print a an execution link.
# In some cases, the rules need to invoke proceedural code.
# The key identifier here is the leading ampersand, which 
# identifies such an execution link.
sub print_exec
{
	my ($clause, $indent) = @_;

	# Pull the three parts out of the clause.
	$clause =~ s/\s*^\&//;
	my ($link, $item1, $item2) = parse_clause($clause);

	# Print a copy of the original clause for reference
	print "$indent;; \&$clause\n";
	print "$indent(ExecutionOutputLink\n";
	print "$indent   (GroundedSchemaNode \"$link\")\n";
	print "$indent   (ListLink\n";
	if ($item1 =~ /^\$/)
	{
		print "$indent      (VariableNode \"$item1\")\n";
	}
	else
	{
		print "$indent      (WordNode \"$item1\")\n";
	}

	if ($item2 =~ /^\$/)
	{
		print "$indent      (VariableNode \"$item2\")\n";
	}
	else
	{
		print "$indent      (WordNode \"$item2\")\n";
	}
	print "$indent   )\n";
	print "$indent)\n";
}

# --------------------------------------------------------------------
# print_word_instances -- print lemma links joining words to instances.
# The core problem here is that the frame rules are written in
# 'general', specifying fixed words if/when needed. However, the 
# actual relex output has WordInstanceNodes, not WordNodes. So we 
# need to somehow tie a specific word instance to a specific word.
#

sub print_word_instances
{
	my ($indent) = @_;
	foreach (keys %word_map)
	{
		if ($_ eq "ccc-cnt-ccc") { next; }

		# Print a lemma link to connect a word instance to its word.
		my $item = $_;

		print "$indent;; word-instance lemma of \"$item\"\n";
		print "$indent(LemmaLink (stv 1.0 1.0)\n";
		# Some unknow word instance, to be fixed by variable.
		# print "$indent   (WordInstanceNode \"$item\")\n";
		my $eyed = $word_map{$item};
		print "$indent   (VariableNode \"$eyed\")\n";
		if ($item =~ /^_%copula/)
		{
			print "$indent   (WordNode \"be\")\n";
		}
		else
		{
			print "$indent   (WordNode \"$item\")\n";
		}
		print "$indent)\n";
	}
}

# --------------------------------------------------------------------
# Parse a single rule, generate the equivalent OpenCog ImplicationLink.
#
# For example, expected input is of the form
#
#    IF _subj(be,$var0) ^ _obj(be,$var1)  THEN ^3_blah($var0, $var1)
#
# This does no syntax checking; if there are any syntax errors in the 
# input, the output is undefined.
#
sub parse_rule
{
	my ($rule) = @_;
	my ($predicate, $implicand) = split(/THEN/, $rule);
	$predicate =~ m/IF(.*)/;
	$predicate = $1;

	my @clauses = split(/\^/, $predicate);

	# Clear the word-to-word-instance map.
	%word_map = ();

	print "(ImplicationLink\n";
	print "   (AndLink\n";
	foreach (@clauses)
	{
		s/^\s*//g;
		my $indent = "      ";

		# Clauses that start with a bang (!) are Not's, to be inverted.
		my $inv = 0;
		if (/^!/)
		{
			$inv = 1;
			print "      (NotLink\n";
			$indent = $indent . "   ";
			s/^!//;
		}

		# Clauses starting with % are "literals"
		if (/^\%/)
		{
			print_link ($_, $indent);
		}
		else
		{
			print_clause ($_, $indent);
		}

		if($inv) { print "      )  ;; NotLink\n"; } # closure to NotLink
	}
	print_word_instances("      ");
	print "   )  ;; AndLink\n";  # closing paren to AndLink

	# We are done with the AndLink. Move on to the implicand.
	while (1)
	{
		# First, strip out the author tag.
		$implicand =~ m/\^\d+_(.*)/;
		$implicand = $1;

		# Now, separate multiple clauses. Unfortunately, the syntax
		# only uses space separator between clauses, so we must
		# parse them to find thier boundaries.
		my @parts = parse_clause($implicand);
		$implicand = pop @parts;
		my $clause;
		if ($#parts == 1)
		{
			my ($p, $i1, $implicand) = @parts;
			$clause = $p . "(" . $i1 . ")";
		}
		else
		{
			my ($p, $i1, $i2, $implicand) = @parts;
			$clause = $p . "(" . $i1 . ", " . $i2 . ")";
		}

		if ($clause =~ /\&scm/)
		{
			print_exec ($clause, "   ");
		}
		elsif ($clause =~ /\&/)
		{
			print_function ($clause, "   ");
		}
		else
		{
			print_clause ($clause, "   ");
		}

		# If the rest of the line starts with ^N_, N a digit, then
		# there's another clause waiting. Loop around again, else quit.
		$implicand =~ s/^\s*//;
		if ($implicand =~ /^\^\d+_/) { next; }
		last;
	}
	print ")\n";
}

# --------------------------------------------------------------------
# Main file-processing loop. Reads from stdin, processes, and prints to stdout.
#
# Tries to pick out rules, one at a time, from input, and pass them to 
# "parse_rule" for processing.
#
# Global vars -- hold the current fragment of a rule, read from input.
my $have_rule = 0;
my $curr_rule = "";

if ($#ARGV < 0) 
{
	die "Usage: rules-to-implications.pl <prefix-string>\n"; 
}

my $rule_name = $ARGV[0];

print "scm\n\n";
# Read from standard input, until theres no more standard input.
#
my $rule_cnt = 0;
while(<STDIN>)
{
	# Ignore comments (actually, reproduce them for ease of debugging)
	if(/^;/)
	{
		print "$_";
		next;
	}

	# New implications are marked by a hash mark at the begining of the
	# line.
	if(/^#/)
	{
		# If we have a rule, parse it.
		if ($have_rule)
		{
			print "(define $rule_name-$rule_cnt\n";
			parse_rule($curr_rule);
			print ")\n";
			$rule_cnt ++;
			$curr_rule = "";
		}

		$have_rule = 1;

		# Strip off the leading hash, the trailing comment, and any newline
		chop;
		m/^# (.*)/;
		my $line = $1;
		my ($line, $rest) = split(/\;/, $line);
		$curr_rule = $curr_rule . $line;
		next;
	}

	# Strip off trailing newline, and any trailing comments; append.
	chop;
	my ($line, $rest) = split(/\;/);
	$curr_rule = $curr_rule . $line;
}

# No more input, parse the final line.
if ($have_rule)
{
	print "(define $rule_name-$rule_cnt\n";
	parse_rule($curr_rule);
	print ")\n";
	$rule_cnt ++;
	$curr_rule = "";
	$have_rule = 0;
}

# Make a list of all the rules.
print "\n(define $rule_name-list (list \n";
for (my $i=0; $i<$rule_cnt; $i++)
{
	print "   $rule_name-$i\n";
}
print "))\n\n";

# Add VariableScope wrappers
for (my $i=0; $i<$rule_cnt; $i++)
{
	print "(define $rule_name-vscope-$i (varscope-wrap-implication $rule_name-$i))\n";
}
print "\n";

#Print list of the var-scoped implications
print "(define $rule_name-vscope-list (list \n";
for (my $i=0; $i<$rule_cnt; $i++)
{
	print "   $rule_name-vscope-$i\n";
}
print "))\n\n";

print "; Processed $rule_cnt rules\n";
print "\n.\nexit\n";
