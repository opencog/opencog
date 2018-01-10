SUBDIRS=words

DICTS=                        \
	4.0.affix                  \
	4.0.constituent-knowledge  \
	4.0.dict                   \
	4.0.dict.m4                \
	4.0.knowledge              \
	4.0.regex

dictdir=$(pkgdatadir)/en
dict_DATA = $(DICTS)

# The make uninstall target should remove directories we created.
uninstall-hook:
	-rmdir $(dictdir)/words
	-rmdir $(dictdir)


# Do NOT install the batch files!
EXTRA_DIST = $(DICTS)         \
	corpus-basic.batch         \
	corpus-biolg.batch         \
	corpus-fixes.batch         \
	corpus-fix-long.batch      \
	corpus-voa.batch           \
	tiny.dict
