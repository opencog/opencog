# For creating images with all the dependencies for development installed.
# Steps:
# 1. ~/opencog/docker/bindmounts.sh
# No need to run the above it is at your own discreation
# 2. docker build -t $USER/opencog-deps .

FROM ubuntu:trusty
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get update 
RUN apt-get -y install software-properties-common python-pip wget

# Install repositories and dependencies
ADD https://raw.githubusercontent.com/opencog/opencog/master/scripts/ocpkg \
    /tmp/install-dependencies-trusty
ADD https://raw.githubusercontent.com/opencog/opencog/master/opencog/python/requirements.txt \
    /tmp/requirements.txt
RUN chmod +x /tmp/install-dependencies-trusty && /tmp/install-dependencies-trusty 
RUN pip install -U -r /tmp/requirements.txt

# Link Parser -- changes often
# Download the current released version of link-grammar.
# The wget gets the latest version w/ wildcard
RUN wget -r --no-parent -nH --cut-dirs=2 http://www.abisource.com/downloads/link-grammar/current/
RUN tar -zxf current/link-grammar-5*.tar.gz
RUN rm -r current
RUN (cd link-grammar-5.*/; mkdir build; cd build; ../configure; make -j6; make install; ldconfig)

# Basic Tools
RUN apt-get -y install git telnet netcat-openbsd

# Environment Variables
## Set Locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog
WORKDIR /home/opencog

# Defaults
## cogserver shell ports
EXPOSE 17001 18001

## REST api
EXPOSE 5000

## ports on which OpenCog's unity3d game communicates with opencog_embodiment
### port from opencog's embodiment code
EXPOSE 16313
### ports from the unity3d game code
EXPOSE 16315 16312

## Default postgresql port
EXPOSE 5432

# Docker defaults
# CMD bash for maintenance only
# ENTRYPOINT should be an application like distcc or buildbot or cogserver
CMD bash

# For images built on this
ONBUILD USER root
