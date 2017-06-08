#! /bin/bash
#
# Delete unwanted article types from the Lithuanian wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# Lithuanian-language sentences.
#
# Copyright (c) 2008 Linas Vepstas <linas@linas.org>

echo "Kategorija:"
find . -name 'Kategorija:*' -print | wc
find . -name 'Vikisritis:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Pagalba:*' -print | wc
echo "Vaizdas:"
find . -name 'Vaizdas:*' -print | wc
find . -name 'Image:*' -print | wc
echo "Šablonas"
find . -name 'Šablonas:*' -print | wc
find . -name 'Vikipedija:*' -print | wc
find . -name 'Vikiprojektas:*' -print | wc
find . -name 'Sąrašas:*' -print | wc

# Must use "find" to accomplish this, since using "rm Kategorija:*"
# leads to an overflow of the command line.

echo "Kategorija:"
time find . -name 'Kategorija:*' -exec rm {} \;
time find . -name 'Vikisritis:*' -exec rm {} \;
time find . -name 'MediaWiki:*' -exec rm {} \;
time find . -name 'Pagalba:*' -exec rm {} \;
# Vaizdas: includes mp3's, ogg's, many different image types
echo "Vaizdas:"
time find . -name 'Vaizdas:*' -exec rm {} \;
time find . -name 'Image:*' -exec rm {} \;
echo "Šablonas"
time find . -name 'Šablonas:*' -exec rm {} \;
time find . -name 'Vikipedija:*' -exec rm {} \;
time find . -name 'Vikiprojektas:*' -exec rm {} \;
time find . -name 'Sąrašas:*' -exec rm {} \;
