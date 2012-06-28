#!/bin/sh
parentdir=`cd ../ ; pwd`
destin_alt=`cd ../../DestinCudaAlt ; pwd`
#java -cp build/classes/ -Djava.library.path="$parentdir" javadestin.VideoExperiment
echo $destin_alt
the_path="${parentdir}:${destin_alt}"
echo "The path: $the_path"
java -cp build/classes/ -Djava.library.path="$the_path" javadestin.Dashboard
