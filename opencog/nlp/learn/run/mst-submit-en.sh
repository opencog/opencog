#!/bin/bash
#
# Batch MST parsing script for English.
# Loop over all the files in 'gamma-pages', sentence-split them
# and submit them for MST parsing.
#
time find gamma-pages -type f -exec ./mst-one.sh en {} localhost 19005 \;
