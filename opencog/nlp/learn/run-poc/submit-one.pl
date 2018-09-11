#! /usr/bin/env perl
#
# submit-one.pl <cogserver-host> <cogserver-port> <observe-cmd> <count-mode> <parameters>
#
# Submit a collection of sentences, one sentence at a time, to the
# cogserver located on host ARGV[0] and port ARGV[1].  The sentences
# are read from standard input, and must be arranged with one sentence
# per line. The are sent to the cogserver using ARGV[2] as the command.
# For word-pair counting, ARGV[2] is "observe-text-mode"
# For disjunct counting, ARGV[2] is "observe-mst-mode"
# ARGV[3] and ARGV[4] (and ARGV[5] for observe-mst-mode) are necessary
# arguments for parameter configuration (see file process-one.sh to see them)
#
# Example usage:
#    cat file | ./submit-one.pl localhost 17001 observe-text-mode "any" 24
#

die "Wrong number of args!" if ($#ARGV < 4);

# Verify that the host and port number are OK.
`nc -z $ARGV[0] $ARGV[1]`;
die "Netcat failed! Bad host or port?" if (0 != $?);

my $netcat = "|nc $ARGV[0] $ARGV[1]";

my $start_time = time();
while (<STDIN>)
{
	if (/<P>/) { next; }
	chop;

	# open(NC, "|nc localhost 17002") || die "nc failed: $!\n";
	open NC, $netcat || die "nc failed: $!\n";
	if ( $ARGV[2] eq "observe-text-mode" )
		{ print NC "($ARGV[2] \"$_\" \"$ARGV[3]\" $ARGV[4])\n"; }
	elsif ( $ARGV[2] eq "observe-mst-mode" )
		{ print NC "($ARGV[2] \"$_\" \"$ARGV[3]\" $ARGV[4] $ARGV[5])\n"; }
	my $elapsed = time() - $start_time;
	print "submit-one (elapsed $elapsed): $_\n";
}
print "Done with article.\n";
