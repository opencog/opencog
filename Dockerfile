# For use with Docker https://www.docker.io/gettingstarted/
#
# Quickstart:
# docker build -t $USER/opencog .
# docker run --name cogserver -d -p 17001:17001 -p 5000:5000 $USER/opencog
# docker logs cogserver

FROM ubuntu:14.04
MAINTAINER David Hart "dhart@opencog.org"
RUN echo "deb http://archive.ubuntu.com/ubuntu trusty main universe" > /etc/apt/sources.list
RUN echo "deb http://archive.ubuntu.com/ubuntu trusty-updates main universe" >> /etc/apt/sources.list
RUN apt-get update 
RUN apt-get -y install software-properties-common wget sudo 

# ocpkg tool to install repositories and dependencies
ADD scripts/ocpkg /octool
RUN chmod +x /octool 
RUN /octool -v -a -d

# hack for libiberty package found in trusty main
RUN sed -i s:"ansidecl.h":\<libiberty/ansidecl.h\>:g /usr/include/bfd.h

# build opencog
ADD . /opencog
RUN ln -s -v /opencog /opencog/src
RUN mkdir -v /opencog/build
RUN /octool -v -b -l /opencog/build

# Start cogserver when container runs
WORKDIR /opencog/build
CMD ["-c",  "/opencog/lib/opencog.conf"]
ENTRYPOINT ["/opencog/build/opencog/server/cogserver"] 
