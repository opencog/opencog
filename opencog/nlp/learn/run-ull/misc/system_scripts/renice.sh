#! /bin/bash
#
# renice.sh
#
# Change scheduling priority for the postgres processes, so that the
# system reacts a bit more nicely when under heavy load.

ps ax |grep postgres |grep pairs | cut -b1-5 | sudo xargs renice 10 
