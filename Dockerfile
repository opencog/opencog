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

# Change line below on rebuild. Will use cache up to this line.
ENV LAST_OS_UPDATE 2015-02-18

RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get -y install software-properties-common python-pip

# Use the ocpkg tool to install repositories and dependencies.
COPY scripts/ocpkg /tmp/install-dependencies-trusty
COPY opencog/python/requirements.txt /tmp/
RUN chmod +x /tmp/install-dependencies-trusty
RUN /tmp/install-dependencies-trusty
RUN pip install -U -r /tmp/requirements.txt

# Tools
RUN apt-get -y install git wget
RUN apt-get -y install rlwrap telnet netcat-openbsd
RUN apt-get -y install gdb python2.7-dbg

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

# Get the things that ockpg didn't get.
# First, sureal needs link-grammar...

# Link Parser -- changes often
# Download the current released version of link-grammar.
# The wget gets the latest version w/ wildcard
RUN wget -r --no-parent -nH --cut-dirs=2 http://www.abisource.com/downloads/link-grammar/current/
RUN tar -zxf current/link-grammar-5*.tar.gz
RUN rm -r current
RUN (cd link-grammar-5.*/; mkdir build; cd build; ../configure; make -j4; sudo make install; sudo ldconfig)

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
