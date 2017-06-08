#! /bin/bash
#
# Delete unwanted article types from the Mandarin/Cantonese wikipedia
# article space. We won't be parsing these; they (mostly) don't contain
# any valid sentences.
#
# Copyright (c) 2008, 2017 Linas Vepstas <linasvepstas@gmail.com>

echo "Category:"
find . -name 'Category:*' -print | wc
find . -name 'MediaWiki:*' -print | wc
find . -name 'Help:*' -print | wc
echo "File:"
find . -name 'File:*' -print | wc
find . -name 'Image:*' -print | wc
find . -name 'Portal:*' -print | wc
find . -name 'Draft:*' -print | wc
echo "Template"
find . -name 'Template:*' -print | wc
find . -name 'Wikipedia:*' -print | wc
find . -name '模組:*' -print | wc
find . -name '模块:*' -print | wc

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
time find . -name '模組:*' -exec rm {} \;
