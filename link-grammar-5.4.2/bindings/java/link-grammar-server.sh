#!/bin/bash
#
# Shell script to start the link-grammar network server.
#
# Usage: java org.linkgrammar.LGService [-verbose] [-threads n] port [language] [dictPath]
# Start a link-grammar parse server on TCP/IP port.  The server returns
# JSON-formatted parse results.  Socket input should be a single sentence
# to parse, proceeded by the identifier "text:".
# 
#   'port'      The TCP port the service should listen to.
#   -verbose    Generate verbose output.
#   -threads    Number of concurrent threads/clients allowed (default 1).
#   'language'  Language abbreviation (en, ru, de, lt, fr, he, tr, any).
#   'dictPath'  Full path to the Link-Grammar dictionaries.
# 
# The below starts the server on port 9000. It the port is omitted,
# help text is printed.  This server can be contacted directly via
# TCP/IP; for example:
#
#   telnet localhost 9000
#
# (Alternately, use netcat instead of telnet). After connecting, type
# in:
#
#   text:  this is an example sentence to parse
#
# The returned bytes will be a JSON message providing the parses of
# the sentence.  By default, the ASCII-art parse of the text is not
# transmitted. This can be obtained by sending messages of the form:
#
#   storeDiagramString:true, text: this is a test.
#
# Putting this all together:
#
#   echo "text:this is a test" | nc localhost 9000
#
# returns the parse; and
#
#   echo "storeDiagramString:true, text:this is a test" | nc localhost 9000
#
# returns the parse and diagram string.

export LANG=en_US.UTF-8

VM_OPTS="-Xmx1024m -Djava.library.path=/usr/lib:/usr/lib/jni:/usr/local/lib:/usr/local/lib/jni"
CLASSPATH="-classpath bin:../../build/bindings/java/bin:../../bindings/java/bin:/usr/share/java/linkgrammar.jar:/usr/local/share/java/linkgrammar.jar"

java $VM_OPTS $CLASSPATH org.linkgrammar.LGService 9000
