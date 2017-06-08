#! /bin/bash
#
# Delete unwanted article types from the Gujarati wikipedia article space.
# We won't be parsing these; they (mostly) don't contain any valid
# Gujarati-language sentences.
#
# Copyright (c) 2015 Ainish Dave <ainishdave@gmail.com>
echo "Category:"
find . -name 'શ્રેણી:*' -print | wc
find . -name 'મીડિયાવિકિ:*' -print | wc
find . -name 'મદદ:*' -print | wc

echo "File:"
find . -name 'Module:*' -print | wc
find . -name 'ચિતર:*' -print | wc
find . -name 'છબી:*' -print | wc

echo "Template"
find . -name 'ઢાંચો:*' -print | wc
find . -name 'વિકિપીડિયા:*' -print | wc

# Must use "find" to accomplish this, since using "rm Category:*"
# leads to an overflow of the command line.
echo "Category:"
time find . -name 'શ્રેણી:*' -exec rm {} \;
time find . -name 'મીડિયાવિકિ:*' -exec rm {} \;
time find . -name 'મદદ:*' -exec rm {} \;
# File: includes mp3's, ogg's, many different image types
echo "File:"
time find . -name 'ચિતર:*' -exec rm {} \;
time find . -name 'છબી:*' -exec rm {} \;
time find . -name 'Module:*' -exec rm {} \;
echo "Template"
time find . -name 'ઢાંચો:*' -exec rm {} \;
time find . -name 'વિકિપીડિયા:*' -exec rm {} \;
