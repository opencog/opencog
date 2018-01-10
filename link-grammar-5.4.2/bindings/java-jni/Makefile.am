#
# Automake file for building liblink-grammar-java.so
#


DEFS = @DEFS@ -DVERSION=\"@VERSION@\"

# $(top_builddir) to pick up autogened link-grammar/link-features.h
AM_CPPFLAGS = -I$(top_srcdir) -I$(top_builddir) $(JAVA_CPPFLAGS) \
	$(WARN_CFLAGS)

lib_LTLIBRARIES = liblink-grammar-java.la

liblink_grammar_java_la_LDFLAGS = -version-info @VERSION_INFO@ -export-dynamic -no-undefined -export-symbols $(srcdir)/link-grammar-java.def

if OS_WIN32
liblink_grammar_java_la_LDFLAGS += -D_JNI_IMPLEMENTATION_ -Wl,--kill-at
endif

liblink_grammar_java_la_LIBADD  = $(top_builddir)/link-grammar/liblink-grammar.la  ${REGEX_LIBS}

if HAVE_HUNSPELL
liblink_grammar_java_la_LIBADD  += ${HUNSPELL_LIBS}
endif

if HAVE_ASPELL
liblink_grammar_java_la_LIBADD  += ${ASPELL_LIBS}
endif

if WITH_SAT_SOLVER
if LIBMINISAT_BUNDLED
liblink_grammar_java_la_LIBADD  += $(top_builddir)/link-grammar/minisat/libminisat.la
else !LIBMINISAT_BUNDLED
liblink_grammar_java_la_LIBADD  += ${MINISAT_LIBS}
endif !LIBMINISAT_BUNDLED
endif

if WITH_CORPUS
liblink_grammar_java_la_LIBADD  += $(top_builddir)/link-grammar/corpus/liblink-corpus.la ${SQLITE3_LIBS}
endif

# Math libraries are needed for floorf, etc.
liblink_grammar_java_la_LIBADD  += -lm

liblink_grammar_java_la_SOURCES =   \
	jni-client.h                     \
	jni-client.c

EXTRA_DIST=                         \
	link-grammar-java.def
