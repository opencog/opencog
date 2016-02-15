#!/bin/bash

if [ $# -ne 2 ]
then
	echo "plot.sh  usage: plot [sw|nsw] [sti|lti|vlti]"
	     "\n plot hebtv hebst"
	exit
fi

UUID=$(eval ./uuids.py dump-$1.data)
eval "./plot.py dump-$1.data $2 $UUID"
