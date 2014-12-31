FROM ubuntu:14.04

RUN apt-get update 
RUN apt-get -y install software-properties-common python-pip

# ocpkg tool to install repositories and dependencies
COPY scripts/ocpkg scripts/install-dependencies-trusty\
    opencog/python/requirements.txt /tmp/
RUN chmod +x /tmp/install-dependencies-trusty
RUN /tmp/install-dependencies-trusty
RUN pip install -U -r /tmp/requirements.txt

# Create and switch user. The user is privileged with no password required
RUN adduser --disabled-password --gecos "OpenCog Developer" opencog
RUN adduser opencog sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER opencog

# Tools
RUN sudo apt-get -y install git
RUN sudo apt-get -y install rlwrap
RUN sudo apt-get -y install telnet

# Environment Variables
## Set Locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

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

# Docker defaults
WORKDIR /home/opencog
CMD bash
