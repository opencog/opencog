#! /bin/bash
#
# Delete unwanted article types from the Tagalog wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# Tagalog-language sentences.
#
# Copyright (c) 2014 Lariena Milambiling

echo "Kategorya:"
find . -name 'Kategorya:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Tulong:*' -print | wc
echo "Talaksan:"
find . -name 'Talaksan:*' -print | wc
find . -name 'Padron:*' -print | wc
echo "Template"
find . -name 'Template:*' -print | wc
find . -name 'Wikipedia:*' -print | wc
find . -name '"Listahan ng mga "*' -print | wc
echo "Translations"
find . -name '(Ingles: *)' -print | wc

# Must use "find" to accomplish this, since using "rm Category:*"
# leads to an overflow of the command line.

echo "Kategorya:"
time find . -name 'Category:*' -exec rm {} \;
time find . -name 'MediaWiki:*' -exec rm {} \;
time find . -name 'Tulong:*' -exec rm {} \;
# File: includes mp3's, ogg's, many different image types
echo "Talaksan:"
time find . -name 'Talaksan:*' -exec rm {} \;
time find . -name 'Padron:*' -exec rm {} \;
echo "Template"
time find . -name 'Template:*' -exec rm {} \;
time find . -name 'Wikipedia:*' -exec rm {} \;
time find . -name '"Listahan ng mga "*' -exec rm {} \;
echo "Translations"
time find . -name '(Ingles: *)' -exec rm {} \;


