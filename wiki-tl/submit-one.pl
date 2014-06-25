#! /usr/bin/env perl


while (<>)
{
	if (/<P>/) { next; }
	chop;

	open(NC, "|nc -w 3 localhost 17001") || die "nc failed: $!\n";
	print NC "scm hush\n(observe-text \"$_\")\n";
	print "submit-one: $_\n";
}
print "Done with article.\n";
