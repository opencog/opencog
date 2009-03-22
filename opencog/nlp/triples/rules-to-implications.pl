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
#    cat some-rule-file.txt | ./rules-to-implications.pl 
#
# This perl script implements a very simple parser. Some day in the future,
# it would proably make sense to re-write this in terms of lex & yacc. Maybe.
#
# Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
# Created in January, March 2009.
#

use strict;

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

	# Pull the three parts out of the clause.
	# The pattern matches "blah(ding,dong)"
	$clause =~ m/\s*([\$!%\-\w]+)\s*\(\s*([\$%\w]+)\s*\,\s*([\$%\w]+)\s*\)(.*)/;
	my $pred = $1;
	my $item1 = $2;
	my $item2 = $3;
	my $rest = $4;

	my @parts;

	# if above failed, then its a double.
	if ($pred eq "")
	{
		# The pattern matches "blah(ding)"
		$clause =~ m/\s*([\$!%\-\w]+)\s*\(\s*([\$%\w]+)\s*\)(.*)/;
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
# Print a single clause. 
# Expects as input a clause, for example, "_subj(be,$var0)", and 
# some whitespace to indent by. Prints, as output, the standard
# OpenCog EvaluationLink equivalent of the clause.
#
sub print_clause
{
	my ($clause, $cnt, $indent) = @_;

	# Pull the three parts out of the clause.
	my ($pred, $item1, $item2) = parse_clause($clause);

	# Print a copy of the original clause for reference
	print "$indent;; $clause\n";
	print "$indent(EvaluationLink\n";
	if ($pred =~ /^\$/)
	{
		print "$indent   (VariableNode \"$pred\")\n";
	}
	else
	{
		print "$indent   (DefinedLinguisticRelationshipNode \"$pred\")\n";
	}

	print "$indent   (ListLink\n";
	if ($item1 =~ /^\$/)
	{
		print "$indent      (VariableNode \"$item1\")\n";
	}
	else
	{
		# Dang, not a word node, but some unknown word-instance node.
		# print "$indent      (WordNode \"$item1\")\n";
		print "$indent      (VariableNode \"\$word-instance-$cnt\")\n";
	}

	if ($item2 =~ /^\$/)
	{
		print "$indent      (VariableNode \"$item2\")\n";
	}
	else
	{
		# Dang, not a word node, but some unknown word-instance node.
		# print "$indent      (WordNode \"$item2\")\n";
		$cnt ++;
		print "$indent      (VariableNode \"\$word-instance-$cnt\")\n";
	}
	print "$indent   )\n";
	print "$indent)\n";
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
	print "$indent($link\n";
	if ($item1 =~ /^\$/)
	{
		print "$indent   (VariableNode \"$item1\")\n";
	}
	else
	{
		print "$indent   (WordNode \"$item1\")\n";
	}

	if ($item2 =~ /^\$/)
	{
		print "$indent   (VariableNode \"$item2\")\n";
	}
	else
	{
		print "$indent   (WordNode \"$item2\")\n";
	}
	print "$indent)\n";
}


# --------------------------------------------------------------------
# print_lemma_link
# Print a lemma link to connect a word instance to its word -- but only
# if neccessary.
#
sub print_lemma_link
{
	my ($clause, $item, $cnt, $indent) = @_;
	if (!($item =~ /^\$/))
	{
		print "$indent;; word-instance lemma of $clause\n";
		print "$indent(LemmaLink\n";
		# Some unknow word instance, to be fixed by variable.
		# print "$indent   (WordInstanceNode \"$item\")\n";
		print "$indent   (VariableNode \"\$word-instance-$cnt\")\n";
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
# print_word_instance -- print lemma links joining words to instances.
# The core problem here is that the frame rules are written in
# 'general', specifying fixed words if/when needed. However, the 
# actual relex output has WordInstanceNodes, not WordNodes. So we 
# need to somehow tie a specific word instance to a specific word.
#
sub print_word_instance
{
	my ($clause, $cnt, $indent) = @_;

	# Pull the three parts out of the clause.
	$clause =~ s/\s*^\%//;
	my ($link, $item1, $item2) = parse_clause($clause);

	# Print a copy of the original clause for reference
	print_lemma_link ($clause, $item1, $cnt, $indent);
	print_lemma_link ($clause, $item2, $cnt+1, $indent);
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

	print "(ImplicationLink\n";
	print "   (AndLink\n";
	my $cnt = 0;
	foreach (@clauses)
	{
		s/^\s*//g;
		my $indent = "      ";

		my $inv = 0;
		if (/^!/)
		{
			$inv = 1;
			print "      (NotLink\n";
			$indent = $indent . "   ";
			s/^!//;
		}
		if (/^\%/)
		{
			print_link ($_, $indent);
		}
		else
		{
			print_clause ($_, $cnt, $indent);
		}

		if($inv) { print "      )  ;; NotLink\n"; } # closure to NotLink
		$cnt += 2;
	}
	$cnt = 0;
	foreach (@clauses)
	{
		print_word_instance ($_, $cnt, "      ");
		$cnt += 2;
	}
	print "   )  ;; AndLink\n";  # closing paren to AndLink

	# We are done with the and link. Move on to the implicand.
	while (1)
	{
		# First, strip out the author tag.
		$implicand =~ m/\^\d+_(.*)/;
		$implicand = $1;

		# Now, separate multiple clauses. Unfortunately, the syntax
		# only uses space separator between clauses, so we must
		# parse them to find thier boundaries.
		my ($p, $i1, $i2, $implicand) = parse_clause($implicand);
		my $clause = $p . "(" . $i1 . ", " . $i2 . ")";
		$implicand = $4;
		print_clause ($clause, 0, "   ");

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

print "scm\n\n";
# Read from standard input, until theres no more standard input.
#
my $rule_cnt = 0;
while(<>)
{
	# Ignore comments (actually, reproduce them for ease of debugging
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
			print "(define frame-rule-$rule_cnt\n";
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
	print "(define frame-rule-$rule_cnt\n";
	parse_rule($curr_rule);
	print ")\n";
	$rule_cnt ++;
	$curr_rule = "";
	$have_rule = 0;
}

print "; Processed $rule_cnt rules\n";
print "\n.\nexit\n";
