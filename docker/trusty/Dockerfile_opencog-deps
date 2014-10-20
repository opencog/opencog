FROM ubuntu:14.04

RUN apt-get update 
RUN apt-get -y install software-properties-common rlwrap

# ocpkg tool to install repositories and dependencies
COPY scripts/ocpkg scripts/install-dependencies-trusty /tmp/
RUN chmod +x /tmp/install-dependencies-trusty
RUN /tmp/install-dependencies-trusty

WORKDIR /home/dev
