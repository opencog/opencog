#! /bin/bash
#
# Delete unwanted article types from the French wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# French-language sentences.
#
# Copyright (c) 2008, 2013 Linas Vepstas <linas@linas.org>

echo "Catégorie:"
find . -name 'Catégorie:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Aide:*' -print | wc
echo "Fichier:"
find . -name 'Fichier:*' -print | wc
find . -name 'Module:*' -print | wc
find . -name 'Modèle:*' -print | wc
echo "Wikipédia:"
find . -name 'Wikipédia:*' -print | wc
find . -name 'Référence:*' -print | wc
find . -name 'Projet:*' -print | wc
find . -name 'Portail:*' -print | wc
find . -name '"Liste de"*' -print | wc

# Must use "find" to accomplish this, since using "rm Catégorie:*"
# leads to an overflow of the command line.

echo "Catégorie:"
time find . -name 'Catégorie:*' -exec rm {} \;
time find . -name 'MediaWiki:*' -exec rm {} \;
time find . -name 'Aide:*' -exec rm {} \;
# Fichier: includes mp3's, ogg's, many different image types
echo "Fichier:"
time find . -name 'Fichier:*' -exec rm {} \;
time find . -name 'Module:*' -exec rm {} \;
time find . -name 'Modèle:*' -exec rm {} \;
echo "Wikipédia:"
time find . -name 'Wikipédia:*' -exec rm {} \;
time find . -name 'Référence:*' -exec rm {} \;
time find . -name 'Projet:*' -exec rm {} \;
time find . -name 'Portail:*' -exec rm {} \;
time find . -name '"Liste de"*' -exec rm {} \;
