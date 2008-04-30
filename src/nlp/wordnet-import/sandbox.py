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
	word = "dog"
	print word
	noundict = nltk.wordnet.dictionaryFor("NOUN");
	wd = noundict['dog'];
	print wd
	print noundict[0];
	print noundict[1];
	print noundict[2];
	print noundict[3];


if __name__ == '__main__':
    import_wn()

__all__ = ["doit"]

