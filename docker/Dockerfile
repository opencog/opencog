# For use with Docker https://www.docker.io/gettingstarted/
#
# docker build -t $USER/opencog-deps .

FROM ubuntu:14.04
MAINTAINER David Hart "dhart@opencog.org"
RUN echo "deb http://hk.archive.ubuntu.com/ubuntu trusty main universe" > /etc/apt/sources.list
RUN apt-get update 
RUN apt-get -y install software-properties-common wget sudo 

# ocpkg tool to install repositories and dependencies
ADD scripts/install-dependencies-trusty
RUN /install-dependencies-trusty

# hack for libiberty package found in trusty main
RUN sed -i s:"ansidecl.h":\<libiberty/ansidecl.h\>:g /usr/include/bfd.h

# CMD bash for maintenance only
# ENTRYPOINT should be an application like distcc or buildbot
CMD /bin/bash
