#!/bin/sh
#
# Batch parsing script.
# Loop over all the files in 'beta-pages', sentence-split them
# and submit them for parsing.
#
time find beta-pages -type f -exec ./ss-one.sh "{}" \;
