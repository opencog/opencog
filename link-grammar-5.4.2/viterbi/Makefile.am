#
# Makefile.am
#
# Master makefile for the viterbi decoder.
#

# Skip this entire directory, if not configured
if WITH_VITERBI

AM_CPPFLAGS = -I.. -I$(top_srcdir) -I$(top_srcdir)/link-grammar \
	$(WARN_CXXFLAGS)

lib_LTLIBRARIES = libvitacog.la

libvitacog_la_SOURCES = \
	atom.cc \
	atom-types.cc \
	compile-base.cc \
	compile.cc \
	compress.cc \
	connect.cc \
	connector-utils.cc \
	disjoin.cc \
	environment.cc \
	garbage.cc \
	parser.cc \
	rewrite.cc \
	upcast.cc \
	word-monad.cc \
	atom.h \
	atom-types.h \
	compile-base.h \
	compile.h \
	compress.h \
	connect.h \
	connector-utils.h \
	disjoin.h \
	environment.h \
	garbage.h \
	parser.h \
	rewrite.h \
	viterbi.h \
	word-monad.h

libvitacog_la_LIBADD = $(top_builddir)/link-grammar/liblink-grammar.la
libvitacog_la_LIBADD += $(LIBGC_LIBS)

# Unit test, to make sure the parser is working correctly.
TESTS = test-env test-disjoin test-parser test-cost

check_PROGRAMS = test-env test-disjoin test-parser test-cost

test_disjoin_SOURCES = test-disjoin.cc
test_parser_SOURCES = test-parser.cc
test_cost_SOURCES = test-cost.cc
test_env_SOURCES = test-env.cc

LDADD = libvitacog.la
LDADD += $(top_builddir)/link-grammar/liblink-grammar.la
LDADD += $(LIBGC_LIBS)

EXTRA_DIST =            \
   README               \
   README.atombase

endif
