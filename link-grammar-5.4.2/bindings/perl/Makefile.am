#
# Makefile for generating perl bindings
#
# Created by Linas Vepstas February 2014
#

SWIG_SOURCES = ../swig/link_grammar.i
SWIG_INCLUDES = ../../link-grammar/link-includes.h
# BUILT_SOURCES = $(top_builddir)/bindings/perl/lg_perl_wrap.cc
BUILT_SOURCES = ../../bindings/perl/lg_perl_wrap.cc

# PERL_EXT_LIB become /usr/local/share/perl/5.14.2 or something like that...
pkgperldir = @PERL_EXT_LIB@

dist_pkgperl_SCRIPTS = \
   $(top_builddir)/bindings/perl/clinkgrammar.pm

if HAVE_SWIG
$(BUILT_SOURCES) $(dist_pkgperl_SCRIPTS): $(SWIG_INCLUDES)
$(BUILT_SOURCES) $(dist_pkgperl_SCRIPTS): $(SWIG_SOURCES)
	$(SWIG) -perl5 -module clinkgrammar -I$(top_srcdir)/link-grammar -I$(top_builddir) -o $@ $<
else
$(BUILT_SOURCES):
	touch $(BUILT_SOURCES)

$(dist_pkgperl_SCRIPTS):
	touch $(dist_pkgperl_SCRIPTS)
endif

# These are packaged in the tarball; make clean should not remove them.
maintainer-clean-local:
	-rm -f $(BUILT_SOURCES) $(dist_pkgperl_SCRIPTS)

# The la MUST have the same name as the pm,
# which MUST be the swig -module name!
# We're calling this clinkgrammar, since its a direct map of the C
# interfaces.
pkgperl_LTLIBRARIES = clinkgrammar.la

clinkgrammar_la_SOURCES = $(BUILT_SOURCES) $(SWIG_SOURCES)

clinkgrammar_la_CPPFLAGS = \
   $(SWIG_PERL_CPPFLAGS)   \
   $(PERL_CFLAGS)          \
   -I$(top_srcdir)         \
   -I$(top_builddir)

clinkgrammar_la_LDFLAGS = -version-info @VERSION_INFO@ $(PERL_LDFLAGS) -module -no-undefined
clinkgrammar_la_LIBADD = $(top_builddir)/link-grammar/liblink-grammar.la

if HAVE_HUNSPELL
clinkgrammar_la_LIBADD  += ${HUNSPELL_LIBS}
endif

if HAVE_ASPELL
clinkgrammar_la_LIBADD  += ${ASPELL_LIBS}
endif


EXTRA_DIST =     \
   README        \
   example.pl    \
   test.pl
