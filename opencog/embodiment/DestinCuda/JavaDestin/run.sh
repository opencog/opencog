#!/bin/sh
parentdir=`cd ../ ; pwd`
destin_alt=`cd ../../DestinCudaAlt ; pwd`
#java -cp build/classes/ -Djava.library.path="$parentdir" javadestin.VideoExperiment
echo $destin_alt
the_path="${parentdir}:${destin_alt}"

java -cp build/classes/ -Djava.library.path="$the_path" javadestin.Dashboard
