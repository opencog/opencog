#
# Automake file for building unit tests.
#

# The tests really require c++11 to build...
CXXFLAGS += -std=c++11

# $(top_builddir) to pick up autogened link-grammar/link-features.h
AM_CPPFLAGS = -I$(top_srcdir) -I$(top_builddir) $(CXX_FLAGS) \
    $(WARN_CXXFLAGS) -DDICTIONARY_DIR=\"$(top_srcdir)\"

# -----------------------------------------------------------
# TESTS declares the tests to actually run;
# check_PROGRAMS are the binaries to build.
check_PROGRAMS = dict-reopen multi-thread mem-leak

if HAVE_JAVA
check_PROGRAMS += multi-java
AM_CPPFLAGS += $(JAVA_CPPFLAGS)
multi_java_SOURCES = multi-java.cc
multi_java_LDADD = -L$(top_builddir)/bindings/java-jni/ -llink-grammar-java -lpthread $(LDADD)
endif

TESTS = $(check_PROGRAMS)

LDFLAGS += $(LINK_CXXFLAGS)

dict_reopen_SOURCES = dict-reopen.cc
multi_thread_SOURCES = multi-thread.cc
mem_leak_SOURCES = mem-leak.cc

LDADD = -L$(top_builddir)/link-grammar/ -llink-grammar
if HAVE_SQLITE
LDADD += $(SQLITE3_LIBS)
endif

multi_thread_LDADD = -lpthread $(LDADD)

if WITH_SAT_SOLVER
if LIBMINISAT_BUNDLED
LDADD  += -L$(top_builddir)/link-grammar/minisat/ -lminisat
else !LIBMINISAT_BUNDLED
LDADD  += ${MINISAT_LIBS}
endif !LIBMINISAT_BUNDLED
endif
