#! /bin/bash
#
# halt.sh
#
# Stop all opencog langauge-learning processes.

# This just sends sighup to the file feeder process.
echo "Shutting down the word-pair text feeder ..."
ps aux |grep pair-submit |grep $(id -nu) | grep -v grep | cut -b10-15 | xargs kill -SIGHUP

sleep 2

echo "Killing guile ..."
# This just sends sighup to the guile process.
ps aux |grep guile |grep $(id -nu) | grep -v grep | cut -b10-15 | xargs kill -SIGHUP
