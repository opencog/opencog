#! /usr/bin/env perl


while (<>)
{
	if (/<P>/) { next; }
	chop;

print "wtf before the nc\n";
	open(NC, "|nc -w 1 localhost 17001") || die "nc failed: $!\n";
print "wtf after the open\n";
	print NC "scm hush\n(observe-text \"$_\")\n";
	print "submit-one: $_\n";
print "wtf before the close\n";
	close(NC);
print "wtf after the close\n";
}
print "Done with article.\n";
