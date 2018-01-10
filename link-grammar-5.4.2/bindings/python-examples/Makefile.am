#
# Makefile for python examples
#
# Created by Linas Vepstas February 2014
#

# Run tests.py if the user says 'make check'
TESTS = tests.py
AM_TESTS_ENVIRONMENT = \
   PYTHONPATH=$(srcdir)/../python:../$(LG_PYDIR):../$(LG_PYDIR)/.libs \
   LINK_GRAMMAR_DATA=$(srcdir)/../../data; \
   export PYTHONPATH LINK_GRAMMAR_DATA;

TEST_EXTENSIONS = .py
PY_LOG_COMPILER = $(PYTHON)
#AM_PY_LOG_FLAGS = -v

EXTRA_DIST =         \
   AUTHORS           \
   LICENSE           \
   README.md         \
   example.py        \
   sentence-check.py \
   parses-en.txt     \
   parses-lt.txt     \
   parses-quotes-en.txt \
   parses-sat-en.txt \
   lg_testutils.py   \
   tests.py


# Installation checks, to be manually done after "make install".
# 1. Check the usability of Python's linkgrammar module directory.
# 2  Check the usability of the link-grammar system language directory.
#
# To run these checks, issue "make installcheck" from a regular user.
installcheck-local:
	$(AM_V_at)for d in . .. ../data ./data; do \
		[ ! -d $$d/en ] ||  { echo "Unexpected dictionary $$d/en"; exit 1; }; \
	done
	PYTHONPATH=$(pythondir) srcdir=$(srcdir) $(PYTHON) $(srcdir)/tests.py ${TESTSFLAGS}
