#!/bin/bash
#
# Batch parsing script for Cantonese.
# Loop over all the files in 'beta-pages', sentence-split them
# and submit them for parsing.
#
time find beta-pages -type f -exec ./ss-one.sh yue {} localhost 17006 \;
# time find beta-pages -type f -exec ./ssx.sh en \"{}\" localhost 17005 \;
# time find beta-pages -type f -exec echo en \"{}\" localhost 17005 \;
