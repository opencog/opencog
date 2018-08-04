#!/usr/bin/perl

$prt = 1;
while(<>)
{
	if (/TypedVariableLink/) {
		$prt = 0;
		next;
	}
	if (/VariableNode/) {
		($junk, $vn) = split("Node ");
		$vn =~ s/\)//;
		chop $vn;
	}

	if (/TypeNode/) {
		($junk, $tn) = split("Node ");
		$tn =~ s/\)//;
		chop $tn;
	}

	if (0 == $prt and /\s+\)$/)
	{
		print "\t\t\t(var-decl $vn $tn)\n";
		$prt = 1;
		next;
	}
	if ($prt) {print "$_"; }
}
