#
# Automake file for building the command line application `link-parser`.
#

DEFS = @DEFS@ -DVERSION=\"@VERSION@\" -DDICTIONARY_DIR=\"$(pkgdatadir)\"

# $(top_builddir) to pick up autogened link-grammar/link-features.h
AM_CPPFLAGS = -I$(top_srcdir) -I$(top_builddir) $(WARN_CFLAGS) \
	$(HUNSPELL_CFLAGS) $(LIBEDIT_CFLAGS)

# -----------------------------------------------------------
# Directives to build the link-parser command-line application
bin_PROGRAMS=link-parser
link_parser_SOURCES = link-parser.c \
                      command-line.c \
                      lg_readline.c \
                      parser-utilities.h \
                      parser-utilities.c \
                      command-line.h \
                      lg_readline.h

link_parser_LDFLAGS = $(LINK_CFLAGS)
link_parser_LDADD = $(top_builddir)/link-grammar/liblink-grammar.la
link_parser_LDADD += $(LIBEDIT_LIBS)
if HAVE_SQLITE
link_parser_LDADD += $(SQLITE3_LIBS)
endif

if WITH_VITERBI
link_parser_LDADD += $(top_builddir)/viterbi/libvitacog.la
link_parser_LDADD += $(LIBGC_LIBS)
endif


# Installation checks, to be manually done after "make install".
# link-parser checks:
# 1. Show the location of its binary.
# 2. Check that it can find the system link-grammar dictionary directory.
# Note the "cd .libs" - it ensures that it will not find "../data".
# To run these checks, issue "make installcheck" from a regular user.
installcheck-local:
	$(AM_V_at)echo; echo -n 'Check location of '; cd .libs; \
		PATH=$(bindir) /usr/bin/which link-parser; echo
	$(AM_V_at)cd .libs; [ "`PATH=$(bindir) /usr/bin/which link-parser`" == $(bindir)/link-parser ] || \
		{ echo -n "Incorrect location for link-parser: "; \
		PATH=$(bindir) /usr/bin/which link-parser; echo; \
		echo "Expected $(bindir)/link-parser"; \
		echo "PATH is ${PATH}"; exit 1; }
	$(AM_V_at)cd .libs; for d in . .. ../data ./data; do \
		[ ! -d $$d/en ] ||  { echo "Unexpected dictionary $$d/en"; exit 1; }; \
	done
	cd .libs; echo "This is a test" | $(bindir)/link-parser
