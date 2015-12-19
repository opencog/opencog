#!/usr/bin/perl

$prt = 1;
$ab = 0;
while(<>)
{
	if (/WordInstanceLink/) {
		$prt = 0;
		next;
	}
	if (/VariableNode/) {
		($junk, $vn) = split("Node ");
		$vn =~ s/\)//;
		chop $vn;
		if (0 == $ab) { $word = $vn;  $ab = 1; }
		else { $parse = $vn; $ab = 0; }
	}

	if (0 == $prt and /\s+\)$/)
	{
		print "\t\t\t(word-in-parse $word $parse)\n";
		$prt = 1;
		next;
	}
	if ($prt) {print "$_"; }
}
