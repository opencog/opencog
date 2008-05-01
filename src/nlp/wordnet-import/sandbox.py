#
# Wordnet to OpenCog prototype sandbox
# Copyright (C) 2008 Linas Vepstas <linas@linas.org>
#

"""Wordnet to OpenCog test sandbox
"""

import nltk;
import nltk.wordnet.dictionary

def import_wn():
	word = "bark"
	print word

	noundict = nltk.wordnet.dictionaryFor("NOUN");
	wd = noundict['bark'];
	print wd

	print "\nduude sense zero"
	print wd[0]

	print "\nduude stuff"
	print wd[0][0]
	# print wd[0][1]
	# print wd[0][2]

	print "\nduude other senses"
	print wd[1]
	print wd[2]
	print wd[3]

	idx = 4440
	print noundict[idx]
	print noundict.getWord("bark")

	# nounsyn = noundict.getSynset(0)


if __name__ == '__main__':
    import_wn()

__all__ = ["doit"]

