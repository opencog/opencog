SUBDIRS=words

DICTS=                        \
	4.0.affix                  \
	4.0.constituent-knowledge  \
	4.0.dict                   \
	4.0.knowledge              \
	4.0.regex                  \
	stem.dict                  \
	morph.dict                 \
	morph-ends.dict            \
	suffix.dict

dictdir=$(pkgdatadir)/ru
dict_DATA = $(DICTS)

# Do NOT install the batch files!
EXTRA_DIST = $(DICTS)         \
	corpus-basic.batch

# The make uninstall target should remove directories we created.
uninstall-hook:
	-rmdir $(dictdir)/words
	-rmdir $(dictdir)
