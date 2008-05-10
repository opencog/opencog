#
# Wordnet to OpenCog synset importer
# Copyright (C) 2008 Linas Vepstas <linas@linas.org>
#

"""Wordnet to OpenCog synset importer
"""

import nltk;
import nltk.wordnet.dictionary

def import_wn():
	print "<!-- OpenCog XML data follows --> "
	noundict = nltk.wordnet.dictionaryFor("NOUN");
	wd = noundict['dog'];
	print wd

	for i in range(0, 4):
		wd = noundict[i+1456]
		print wd
		ss = wd.synsets()  # returns a list of sysnsets ss
		for sens in ss:
			print sens


if __name__ == '__main__':
    import_wn()

__all__ = ["doit"]

