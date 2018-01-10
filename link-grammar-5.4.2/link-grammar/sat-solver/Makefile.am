if LIBMINISAT_BUNDLED
BUNDLED_INCLUDES = -I$(top_srcdir)/link-grammar/minisat -I$(top_srcdir)/link-grammar/minisat/minisat
endif

AM_CPPFLAGS = -I$(top_srcdir) -I$(top_srcdir)/link-grammar -I$(top_builddir) \
				  ${MINISAT_INCLUDES} $(ZLIB_CPPFLAGS) $(BUNDLED_INCLUDES) $(WARN_CXXFLAGS)

noinst_LTLIBRARIES = libsat-solver.la

libsat_solver_la_SOURCES = \
	clock.hpp         \
	fast-sprintf.cpp  \
	fast-sprintf.hpp  \
	guiding.hpp       \
	matrix-ut.hpp     \
	sat-encoder.cpp   \
	sat-encoder.hpp   \
	sat-encoder.h     \
	trie.hpp          \
	util.cpp          \
	util.hpp          \
	variables.cpp     \
	variables.hpp     \
	word-tag.cpp      \
	word-tag.hpp
