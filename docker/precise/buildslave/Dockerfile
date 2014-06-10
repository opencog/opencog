# For use with Docker https://www.docker.io/gettingstarted/
#
# docker build -t $USER/opencog-precise-buildslave .
# docker run -d -e BUILDSLAVE_NAME=$HOSTNAME -e BUILDSLAVE_PASSWD=foobar $USER/opencog-precise-buildslave

FROM ubuntu:12.04
MAINTAINER David Hart "dhart@opencog.org"

RUN echo "deb http://hk.archive.ubuntu.com/ubuntu precise main universe" > /etc/apt/sources.list && \
    echo "deb http://hk.archive.ubuntu.com/ubuntu precise-updates main universe" >> /etc/apt/sources.list && \
    DEBIAN_FRONTEND=noninteractive apt-get update --assume-yes && \
    DEBIAN_FRONTEND=noninteractive apt-get install \
                                                   python-software-properties \
                                                   locales \
                                                   sudo \
                                                   wget \
                                                   git \
                                                   unzip \
                                                   less \
                                                   vim \
                                                   byobu \
                                                   mosh \
                                           --assume-yes

RUN localedef -c -i en_US -f UTF-8 en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# ocpkg tool to install repositories and dependencies
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg /octool
RUN chmod ugo+x /octool && /octool -a -d

RUN apt-get -y install python-pip python-dev

RUN pip install buildbot-slave==0.8.8

RUN adduser --disabled-password -uid 1099 --gecos "Buildbot,,," --home /buildbot buildbot

RUN su buildbot sh -c "buildslave create-slave --umask=022 /buildbot buildbot.opencog.org:9989 BUILDSLAVE_NAME BUILDSLAVE_PASSWD"

ADD opencog /buildbot/opencog_master_precise/build
RUN chown -R buildbot:buildbot /buildbot
WORKDIR /buildbot/opencog_master_precise/build

RUN git remote rm origin
RUN git remote add origin git://github.com/opencog/opencog

# set buildslave admin and host info
RUN echo "David Hart <dhart@opencog.org>" > /buildbot/info/admin && \
    grep "model name" /proc/cpuinfo | head -1 | cut -d ":" -f 2 | tr -d " " > /buildbot/info/host && \
    grep DISTRIB_DESCRIPTION /etc/lsb-release | cut -d "=" -f 2 | tr -d "\"" >> /buildbot/info/host

CMD su buildbot sh -c "\
sed -i s:BUILDSLAVE_NAME:$BUILDSLAVE_NAME:g /buildbot/buildbot.tac && \
sed -i s:BUILDSLAVE_PASSWD:$BUILDSLAVE_PASSWD:g /buildbot/buildbot.tac && \
BUILDSLAVE_PASSWD=xxxxxx /usr/local/bin/buildslave start --nodaemon /buildbot"
