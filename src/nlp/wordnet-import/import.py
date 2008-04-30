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
	print dict["dog"];


if __name__ == '__main__':
    import_wn()

__all__ = ["doit"]

