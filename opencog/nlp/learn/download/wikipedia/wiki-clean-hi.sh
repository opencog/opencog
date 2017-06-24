#! /bin/bash
#
# Delete unwanted article types from the Hindi wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# Hindi-language sentences.
#
# Copyright (c) 2015 Ainish Dave <ainishdave@gmail.com>
echo "Category:"
find . -name 'श्रेणी:*' -print | wc
find . -name 'मीडियाविकि:*' -print | wc
find . -name 'सहायता:*' -print | wc
echo "File:"
find . -name 'चित्र:*' -print | wc
find . -name 'छवि:*' -print | wc
find . -name 'Modules:*' -print | wc
echo "Template"
find . -name 'सांचा:*' -print | wc
find . -name 'विकिपीडिया:*' -print | wc


# Must use "find" to accomplish this, since using "rm Category:*"
# leads to an overflow of the command line.
echo "Category:"
time find . -name 'श्रेणी:*' -exec rm {} \;
time find . -name 'मीडियाविकि:*' -exec rm {} \;
time find . -name 'सहायता:*' -exec rm {} \;

# File: includes mp3's, ogg's, many different image types
echo "File:"
time find . -name 'चित्र:*' -exec rm {} \;
time find . -name 'छवि:*' -exec rm {} \;
time find . -name 'Modules:*' -exec rm {} \;

echo "Template"
time find . -name 'सांचा:*' -exec rm {} \;
time find . -name 'विकिपीडिया:*' -exec rm {} \;
