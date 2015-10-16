#
# Primary OpenCog Dockerfile
#
# Usage:  docker build -t $USER/opencog .
#         docker run --name="my_cog" -p 17001:17001 -p 18001:18001 -it $USER/opencog
#         docker exec -it my_cog bash
#
FROM ubuntu:14.04
MAINTAINER David Hart "dhart@opencog.org"
MAINTAINER Linas Vepstas "linasvepstas@gmail.com"

RUN apt-get update ; apt-get -y upgrade ; apt-get -y autoclean
RUN apt-get -y install software-properties-common wget rlwrap \
                       telnet netcat-openbsd git gdb python2.7-dbg

# Use the ocpkg tool to install repositories and dependencies.
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg \
    /tmp/octool
RUN chmod 755 /tmp/octool && /tmp/octool -rdpcalv

# Copy the .gdbinit file so we can debug the CogServer
COPY scripts/.gdbinit ~/.gdbinit

# Environment Variables
## Set Locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Create and switch user. The user is privileged, with no password required.
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog

RUN mkdir /home/opencog/src
WORKDIR /home/opencog/src

# Change line below on rebuild. Will use cache up to this line.
ENV LAST_SOFTWARE_UPDATE 2015-02-18

# Now, OpenCog itself.
RUN git clone https://github.com/opencog/opencog
RUN (cd opencog; mkdir build; cd build; cmake ..; make -j4)

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
CMD bash
