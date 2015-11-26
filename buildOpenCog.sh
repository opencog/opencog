#!/bin/bash

cd ../opencog
b="$(git rev-parse --abbrev-ref HEAD)"
echo "$b"
if ["$b" != "OpenCogMineCraft"]
then
    echo "Creating new Branch"
    git stash
    git checkout -b "OpenCogMineCraft"
fi

cd ../opencog-to-minecraft

cp -r opencog/ ../.
cd ../opencog
mkdir build
cd build
cmake ..
make
