#!/bin/bash
#
# Batch parsing script for English.
# Loop over all the files in 'beta-pages', sentence-split them
# and submit them for parsing.
#
time find beta-pages -type f -exec ./ss-one.sh en \"{}\" localhost 17005\;
