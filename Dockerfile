# For use with Docker https://www.docker.io/gettingstarted/
#
# Quickstart:
# docker build -t $USER/opencog .
# docker run --name cogserver -d -p 17001:17001 -p 5000:5000 $USER/opencog
# docker logs cogserver
# sudo lxc-attach -n `docker inspect cogserver | grep '"ID"' | sed 's/[^0-9a-z]//g'` /bin/bash

FROM ubuntu:12.04
MAINTAINER David Hart "dhart@opencog.org"
RUN echo "deb http://archive.ubuntu.com/ubuntu precise main universe" > /etc/apt/sources.list
RUN apt-get update 
RUN apt-get -y install python-software-properties wget sudo 

# Get ocpkg script, replace wget with ADD when caching feature is added
ADD . /opencog
ADD https://raw.github.com/opencog/ocpkg/master/ocpkg /opencog/
RUN chmod -v u+x /opencog/ocpkg && \
    ln -s -v /opencog/ocpkg /opencog/octool && \
    ln -s -v /opencog /opencog/src && \
    mkdir -v -p /opencog/bin 

# Add repositories, install dependencies, build; still depends on opencog/bin
RUN /opencog/octool -v -a -d
RUN /opencog/octool -v -b -l /opencog/bin

# Start cogserver when container runs
WORKDIR /opencog/bin
CMD ["-c",  "/opencog/lib/opencog.conf"]
ENTRYPOINT ["/opencog/bin/opencog/server/cogserver"] 
