#! /bin/bash
#
# halt.sh
#
# Stop all opencog langauge-learning processes.

# This just sends sighup to the file feeder process.
echo "Shutting down the text feeder ..."
ps aux |grep wiki-ss |grep $(id -nu) | grep -v grep | cut -b10-15 | xargs kill -SIGHUP

echo "Killing guile ..."
# This just sends sighup to the guile process.
ps aux |grep guile |grep $(id -nu) | grep -v grep | cut -b10-15 | xargs kill -SIGHUP
