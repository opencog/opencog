#
# Makefile.am
# Master automake file for building the corpus-statistics tools
#

DEFS = @DEFS@ -DVERSION=\"@VERSION@\" -DDICTIONARY_DIR=\"$(pkgdatadir)\"

AM_CPPFLAGS = -I$(top_srcdir)/link-grammar $(WARN_CFLAGS) $(SQLITE3_CFLAGS)

if WITH_CORPUS
lib_LTLIBRARIES = liblink-corpus.la
endif

# Undefined symbols are allowed, as otherwise there is a circular
# dependenincy on liblink-grammar.
# liblink_corpus_la_LDFLAGS = -no-undefined
liblink_corpus_la_LDFLAGS = $(LINK_CFLAGS)

liblink_corpus_la_LIBADD = ${SQLITE3_LIBS}

liblink_corpus_la_SOURCES = \
	cluster.h                \
	cluster.c                \
	corpus.h                 \
	corpus.c


EXTRA_DIST=                               \
	README

# Don't build the tools by default; only the maintainer (me) needs
# to run this tool.
#
# Specifying the binaries here leads to circular Make dependencies
# which I am too lazy to fix.  To build the tool, you need to uncomment
# the below, then run a two-step build process:
#    autogen.sh ; make
#    autogen.sh --enable-corpus-stats ; make
#
# noinst_PROGRAMS=cluster-pop sniff

cluster_pop_SOURCES = cluster-pop.c
cluster_pop_LDADD = ${SQLITE3_LIBS}
cluster_pop_LDADD += $(top_builddir)/link-grammar/liblink-grammar.la

sniff_SOURCES = sniff.c
sniff_LDADD = ${SQLITE3_LIBS}
