#!/bin/bash

echo "pair counting"
./text-process.sh pairs en

echo "MI-calculating"
source ./config/params.txt
echo -e "(load \"compute-mi.scm\")" | nc localhost 17005
echo -e "(comp-mi \"$cnt_mode\")" | nc localhost 17005

echo "MST-parsing"
time find gamma-pages -type f -exec ./process-one.sh mst en {} localhost 17005 \;

echo -e "(load \"export-mi.scm\")" | nc localhost 17005
echo -e "(export-mi \"$cnt_mode\")" | nc localhost 17005

