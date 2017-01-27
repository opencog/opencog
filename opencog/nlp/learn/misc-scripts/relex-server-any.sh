#!/bin/bash
#
# relex-server-any.sh: read from socket, generate opencog output.
#
# This script starts a RelEx server that listens for plain-text input
# (for the ANY langauge) on port 4445.  The any langauge generates
# random parses. The resulting random parse is returned in opencog format
# on the same socket. The end of the parse is demarcated with an
# ; END OF SENTENCE token.
#
# It is intended that this server be used entirely from within OpenCog
# (primarily for syntax learning), to parse text. It is not intended
# for general, manual use.
#
# Example usage:
#    ./relex-server-any.sh &
#    telnet localhost 4445
#    This is a test
#    ^]q
#
export LANG=en_US.UTF-8

VM_OPTS="-Xmx1024m"

RELEX_OPTS="\
	-Djava.library.path=/usr/lib:/usr/lib/jni:/usr/local/lib:/usr/local/lib/jni \
	-Drelex.algpath=data/relex-semantic.algs \
	-Dwordnet.configfile=data/wordnet/file_properties.xml \
	"

CLASSPATH="-classpath \
/usr/local/share/java/relex.jar:\
/usr/local/share/java/opennlp-tools-1.5.3.jar:\
/usr/local/share/java/opennlp-tools-1.5.0.jar:\
/usr/local/share/java/maxent-3.0.3.jar:\
/usr/local/share/java/maxent-3.0.0.jar:\
/usr/local/share/java/maxent-2.5.2.jar:\
/usr/local/share/java/trove.jar:\
/usr/local/share/java/jwnl-1.4rc2.jar:\
/usr/local/share/java/jwnl.jar:\
/usr/share/java/commons-logging.jar:\
/usr/share/java/gnu-getopt.jar:\
/usr/share/java/linkgrammar.jar:\
/usr/local/share/java/linkgrammar.jar:\
"

# Return ANY langauge parses on default port 4445.
java $VM_OPTS $RELEX_OPTS $CLASSPATH relex.Server --link --lang any -n 16 --port 4445

# Like the above, but sents the output to a different host, instead of
# replying on the same socket.
# java $VM_OPTS $RELEX_OPTS $CLASSPATH relex.Server --port 4242 --host somewhere.com:17001
