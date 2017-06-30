#! /bin/bash
#
# Delete unwanted article types from the Polish wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# Polish-language sentences.
#
# Copyright (c) 2008, 2013 Linas Vepstas <linas@linas.org>

echo "Kategoria:"
find . -name 'Kategoria:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Pomoc:*' -print | wc
echo "Plik:"   # plik is file, and obraz is image
find . -name 'Plik:*' -print | wc
find . -name 'Obraz:*' -print | wc
echo "Szablon"
find . -name 'Szablon:*' -print | wc
find . -name 'Wikipedia:*' -print | wc
find . -name 'Wikiprojekt:*' -print | wc
find . -name 'Portal:*' -print | wc
# find . -name '"Lista  odcinków"*' -print | wc

# Must use "find" to accomplish this, since using "rm Kategoria:*"
# leads to an overflow of the command line.

echo "Kategoria:"
time find . -name 'Kategoria:*' -exec rm {} \;
time find . -name 'MediaWiki:*' -exec rm {} \;
time find . -name 'Pomoc:*' -exec rm {} \;
# Plik: includes mp3's, ogg's, many different image types
echo "Plik:"
time find . -name 'Plik:*' -exec rm {} \;
time find . -name 'Obraz:*' -exec rm {} \;
echo "Szablon"
time find . -name 'Szablon:*' -exec rm {} \;
time find . -name 'Wikipedia:*' -exec rm {} \;
time find . -name 'Wikiprojekt:*' -exec rm {} \;
time find . -name 'Portal:*' -exec rm {} \;
# time find . -name 'Lista:*' -exec rm {} \;
