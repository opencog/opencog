#!/bin/bash




UUID=$(eval ./uuids.py dump-$1.data)
eval "./plot.py dump-$1.data $2 $UUID"
