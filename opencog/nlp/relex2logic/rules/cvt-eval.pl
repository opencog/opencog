#!/usr/bin/perl

$prt = 1;
$ab = 0;
$skip = 1;
while(<>)
{
	if (/EvaluationLink/) {
		$prt = 0;
		$skip = 1;
		next;
	}
	if (/DefinedLinguisticRelationshipNode/) {
		($junk, $rel) = split("Node ");
		$rel =~ s/\)//;
		chop $rel;
	}

	if (/VariableNode/) {
		($junk, $vn) = split("Node ");
		$vn =~ s/\)//;
		chop $vn;
		if (0 == $ab) { $hed = $vn;  $ab = 1; }
		else { $dep = $vn; $ab = 0; }
	}

	if (0 == $prt and /\s+\)$/)
	{
		if (1 == $skip) { $skip = 0; next; }
		print "\t\t\t(dependency $rel $hed $dep)\n";
		$prt = 1;
		next;
	}
	if ($prt) {print "$_"; }
}
