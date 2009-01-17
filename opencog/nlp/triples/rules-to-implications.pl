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

use strict;

sub parse_clause
{
	my ($clause) = @_;

	# Pull the three parts out of the clause.
	$clause =~ m/\s*([\$\w]+)\s*\(\s*([\$\w]+)\s*\,\s*([\$\w]+)\s*\)(.*)/;
	my $pred = $1;
	my $item1 = $2;
	my $item2 = $3;
	my $rest = $4;

	my @parts = ($pred, $item1, $item2, $rest);
	@parts;
}

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
	if (!($item1 =~ /^\$/))
	{
		print "$indent;; word-instance lemma of $clause\n";
		print "$indent(LemmaLink\n";
		# Some unknow word instance, to be fixed by variable.
		# print "$indent   (WordInstanceNode \"$item1\")\n";
		print "$indent   (VariableNode \"\$word-instance-$cnt\")\n";
		print "$indent   (WordNode \"$item1\")\n";
		print "$indent)\n";
	}

	if (!($item2 =~ /^\$/))
	{
		print "$indent;; word-instance lemma of $clause\n";
		print "$indent(LemmaLink\n";
		$cnt ++;
		# Some unknow word instance, to be fixed by variable.
		print "$indent   (VariableNode \"\$word-instance-$cnt\")\n";
		print "$indent   (WordNode \"$item2\")\n";
		print "$indent)\n";
	}
}

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
		if (/^\%/)
		{
			print_link ($_, "      ");
		}
		else
		{
			print_clause ($_, $cnt, "      ");
		}
		$cnt += 2;
	}
	$cnt = 0;
	foreach (@clauses)
	{
		print_word_instance ($_, $cnt, "      ");
		$cnt += 2;
	}
	print "   )\n";

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

		# strip off the leading hash, the traling comment, and any newline
		chop;
		m/^# (.*)/;
		my $line = $1;
		my ($line, $rest) = split(/\;/, $line);
		$curr_rule = $curr_rule . $line;
		next;
	}

	# strip off trailing newline, and any trailing comments; append.
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
