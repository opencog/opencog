#! /bin/bash
#
# Delete unwanted article types from the English wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# English-language sentences.
#
# Copyright (c) 2008 Linas Vepstas <linas@linas.org>

echo "Category:"
find . -name 'Category:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Help:*' -print | wc
echo "File:"
find . -name 'File:*' -print | wc
find . -name 'Image:*' -print | wc
echo "Template"
find . -name 'Template:*' -print | wc
find . -name 'Wikipedia:*' -print | wc
find . -name '"List of "*' -print | wc
find . -name '"Lists of "*' -print | wc

# Must use "find" to accomplish this, since using "rm Category:*"
# leads to an overflow of the command line.

echo "Category:"
time find . -name 'Category:*' -exec rm {} \;
time find . -name 'MediaWiki:*' -exec rm {} \;
time find . -name 'Help:*' -exec rm {} \;
# File: includes mp3's, ogg's, many different image types
echo "File:"
time find . -name 'File:*' -exec rm {} \;
time find . -name 'Image:*' -exec rm {} \;
echo "Template"
time find . -name 'Template:*' -exec rm {} \;
time find . -name 'Wikipedia:*' -exec rm {} \;
time find . -name '"List of "*' -exec rm {} \;
time find . -name '"Lists of "*' -exec rm {} \;
