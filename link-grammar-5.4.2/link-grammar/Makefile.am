#
# Automake file for building liblink-grammar.so
#

SUBDIRS=

if LIBMINISAT_BUNDLED
MINISAT_DIR = minisat
endif

if WITH_SAT_SOLVER
SUBDIRS += $(MINISAT_DIR) sat-solver
endif

if WITH_CORPUS
SUBDIRS += corpus
endif

if OS_WIN32

install-libtool-import-lib:
	$(INSTALL) .libs/liblink-grammar.dll.a $(DESTDIR)$(libdir)

uninstall-libtool-import-lib:
	-rm $(DESTDIR)$(libdir)/liblink-grammar.dll.a
else
install-libtool-import-lib:
uninstall-libtool-import-lib:
endif

DEFS = @DEFS@ -DVERSION=\"@VERSION@\" -DDICTIONARY_DIR=\"$(pkgdatadir)\"

# $(top_builddir) to pick up autogened link-grammar/link-features.h
AM_CPPFLAGS = -I$(top_srcdir) -I$(top_builddir) $(WARN_CFLAGS) \
	$(HUNSPELL_CFLAGS)

post-process/pp_lexer.lo: AM_CPPFLAGS += -I$(top_srcdir)/link-grammar/post-process
MAINTAINERCLEANFILES = $(top_srcdir)/link-grammar/post-process/pp_lexer.c

lib_LTLIBRARIES = liblink-grammar.la

EXTRA_liblink_grammar_la_DEPENDENCIES = $(srcdir)/link-grammar.def

liblink_grammar_la_LDFLAGS = -version-info @VERSION_INFO@ \
	-export-dynamic -no-undefined \
	-export-symbols $(srcdir)/link-grammar.def \
	$(LINK_CFLAGS)

if OS_WIN32
liblink_grammar_la_LDFLAGS += -D_JNI_IMPLEMENTATION_ -Wl,--kill-at
endif

liblink_grammar_la_LIBADD  =  ${REGEX_LIBS}

if HAVE_HUNSPELL
liblink_grammar_la_LIBADD  +=  ${HUNSPELL_LIBS}
endif

if HAVE_ASPELL
liblink_grammar_la_LIBADD  +=  ${ASPELL_LIBS}
endif

if HAVE_SQLITE
liblink_grammar_la_LIBADD  +=  ${SQLITE3_LIBS}
endif

if WITH_SAT_SOLVER
liblink_grammar_la_LIBADD  += $(top_builddir)/link-grammar/sat-solver/libsat-solver.la
if LIBMINISAT_BUNDLED
liblink_grammar_la_LIBADD  += $(top_builddir)/link-grammar/minisat/libminisat.la
else !LIBMINISAT_BUNDLED
liblink_grammar_la_LIBADD  += ${MINISAT_LIBS}
endif !LIBMINISAT_BUNDLED
endif WITH_SAT_SOLVER

if WITH_CORPUS
liblink_grammar_la_LIBADD  += $(top_builddir)/link-grammar/corpus/liblink-corpus.la ${SQLITE3_LIBS}
endif

# Math libraries are needed for floorf, etc.
liblink_grammar_la_LIBADD  += -lm

liblink_grammar_la_SOURCES =        \
	api.c                            \
	connectors.c                     \
	dict-common/dict-common.c        \
	dict-common/dict-impl.c          \
	dict-common/dict-utils.c         \
	dict-common/file-utils.c         \
	dict-common/idiom.c              \
	dict-common/print-dict.c         \
	dict-common/regex-morph.c        \
	dict-file/dictionary.c           \
	dict-file/read-dict.c            \
	dict-file/read-regex.c           \
	dict-file/word-file.c            \
	dict-sql/read-sql.c              \
	disjunct-utils.c                 \
	error.c                          \
	linkage/analyze-linkage.c        \
	linkage/freeli.c                 \
	linkage/linkage.c                \
	linkage/lisjuncts.c              \
	linkage/sane.c                   \
	linkage/score.c                  \
	parse/count.c                    \
	parse/extract-links.c            \
	parse/fast-match.c               \
	parse/histogram.c                \
	parse/parse.c                    \
	parse/preparation.c              \
	parse/prune.c                    \
	post-process/constituents.c      \
	post-process/post-process.c      \
	post-process/pp_knowledge.c      \
	post-process/pp_lexer.l          \
	post-process/pp_linkset.c        \
	prepare/build-disjuncts.c        \
	prepare/expand.c                 \
	prepare/exprune.c                \
	print/print.c                    \
	print/print-util.c               \
	print/wcwidth.c                  \
	resources.c                      \
	string-set.c                     \
	tokenize/anysplit.c              \
	tokenize/spellcheck-aspell.c     \
	tokenize/spellcheck-hun.c        \
	tokenize/regex-tokenizer.c       \
	tokenize/tokenize.c              \
	tokenize/wordgraph.c             \
	utilities.c                      \
	                                 \
	api-structures.h                 \
	api-types.h                      \
	connectors.h                     \
	dict-common/dict-affix.h         \
	dict-common/dict-api.h           \
	dict-common/dict-common.h        \
	dict-common/dict-defines.h       \
	dict-common/dict-impl.h          \
	dict-common/dict-structures.h    \
	dict-common/dict-utils.h         \
	dict-common/file-utils.h         \
	dict-common/idiom.h              \
	dict-common/regex-morph.h        \
	dict-file/read-dict.h            \
	dict-file/read-regex.h           \
	dict-file/word-file.h            \
	dict-sql/read-sql.h              \
	disjunct-utils.h                 \
	error.h                          \
	externs.h                        \
	lg_assert.h                      \
	link-includes.h                  \
	linkage/analyze-linkage.h        \
	linkage/linkage.h                \
	linkage/lisjuncts.h              \
	linkage/sane.h                   \
	linkage/score.h                  \
	parse/count.h                    \
	parse/extract-links.h            \
	parse/fast-match.h               \
	parse/histogram.h                \
	parse/parse.h                    \
	parse/preparation.h              \
	parse/prune.h                    \
	post-process/post-process.h      \
	post-process/pp_knowledge.h      \
	post-process/pp_lexer.h          \
	post-process/pp_linkset.h        \
	post-process/pp-structures.h     \
	prepare/build-disjuncts.h        \
	prepare/expand.h                 \
	prepare/exprune.h                \
	print/print.h                    \
	print/print-util.h               \
	print/wcwidth.h                  \
	resources.h                      \
	string-set.h                     \
	tokenize/anysplit.h              \
	tokenize/spellcheck.h            \
	tokenize/regex-tokenizer.h       \
	tokenize/tok-structures.h        \
	tokenize/tokenize.h              \
	tokenize/word-structures.h       \
	tokenize/wordgraph.h             \
	utilities.h

liblink_grammar_includedir = $(includedir)/link-grammar
liblink_grammar_include_HEADERS =   \
	link-features.h                  \
	link-includes.h                  \
	dict-common/dict-api.h           \
	dict-common/dict-structures.h

EXTRA_DIST=                         \
	link-grammar.def                 \
	README.md                        \
	dict-sql/dict.sql                \
	dict-sql/demo.sql                \
	dict-sql/README.md               \
	tokenize/README.md

# -----------------------------------------------------------
install-data-local: install-libtool-import-lib

uninstall-local: uninstall-libtool-import-lib
