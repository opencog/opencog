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

use strict;

my $have_rule = 0;
my $curr_rule = "";

sub print_clause
{
	my ($clause, $indent) = @_;

	# Pull the three parts out of the clause.
	m/\s*([\$\w]+)\s*\(\s*([\$\w]+)\s*\,\s*([\$\w]+)\s*\)/;
	my $pred = $1;
	my $item1 = $2;
	my $item2 = $3;

	# Print a copy of the original clause for reference
	print "$indent;; $_\n";
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

sub parse_rule
{
	my ($rule) = @_;
	my ($predicate, $implicand) = split(/THEN/, $rule);
	$predicate =~ m/IF(.*)/;
	$predicate = $1;

	my @clauses = split(/\^/, $predicate);

	print "(ImplicationLink\n";
	print "   (AndLink\n";
	foreach (@clauses)
	{
		print_clause ($_, "      ");
	}
	print "   )\n";

	# We are done with the and link. Move on to the implicand.
	# First, strip out the author tag.
	while (1)
	{
		$implicand =~ m/\^\d+_(.*)/;
		$implicand = $1;
	print "hello $implicand\n";
		print_clause ($implicand, "   ");
		last;
	}
	print ")\n";
}

while(<>)
{
	# Ignore comments
	if(/^;/)
	{
		next;
	}
	# New implications are marked by a hash mark at the begining of the
	# line.
	if(/^#/)
	{
		# If we have a rule, parse it.
		if ($have_rule)
		{
			parse_rule($curr_rule);
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

# no more input, parse the final line.
if ($have_rule)
{
	parse_rule($curr_rule);
	$curr_rule = "";
	$have_rule = 0;
}
