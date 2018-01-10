#
# Makefile for the Java wrapper jar file
#

JAVA_SRCS =                             \
   org/linkgrammar/JSONReader.java      \
   org/linkgrammar/JSONUtils.java       \
   org/linkgrammar/LGConfig.java        \
   org/linkgrammar/LGRemoteClient.java  \
   org/linkgrammar/LGService.java       \
   org/linkgrammar/Linkage.java         \
   org/linkgrammar/LinkGrammar.java     \
   org/linkgrammar/Link.java            \
   org/linkgrammar/ParseResult.java

# Build and install the jar file in $(PREFIX)/share/java ...
linkgrammar-@VERSION@.jar: $(JAVA_SRCS)
	ant

if HAVE_ANT
# distclean-generic will delete build.xml, so clean-local must run
# first.... but then, build.xml doesn't get removed :-(
distclean-generic: clean-local
distclean-local: clean-local
	-rm -f build.xml

# Let ant delete the files that ant creates.
clean-local:
	ant clean
	-rm -f *.jar
endif

javadir=$(datadir)/java
java_DATA = linkgrammar-@VERSION@.jar

if OS_WIN32
LN_SF = cp
else
LN_SF = ln -sf
endif

install-data-hook:
	${LN_SF} ${javadir}/${java_DATA} ${DESTDIR}/${javadir}/linkgrammar.jar

uninstall-hook:
	-rm ${DESTDIR}/${javadir}/linkgrammar.jar


EXTRA_DIST =                         \
   README                            \
   link-grammar-server.sh            \
   ${JAVA_SRCS}
