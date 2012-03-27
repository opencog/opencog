#!/bin/sh
parentdir=`cd ../ ; pwd`
#java -cp build/classes/ -Djava.library.path="$parentdir" javadestin.JavaDestin 
#java -cp build/classes/ -Djava.library.path="$parentdir" javadestin.VideoExperiment
java -cp build/classes/ -Djava.library.path="$parentdir" javadestin.Dashboard
