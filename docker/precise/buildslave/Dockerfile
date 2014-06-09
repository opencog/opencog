# For use with Docker https://www.docker.io/gettingstarted/
#
# docker build -t $USER/opencog-precise-buildslave .
# docker run -d -e BUILDSLAVE_NAME=$HOSTNAME -e BUILDSLAVE_PASSWD=foobar $USER/opencog-precise-buildslave

MAINTAINER David Hart "dhart@opencog.org"
FROM ubuntu:12.04

RUN apt-get update 
RUN apt-get -y install python-software-properties sudo 

# ocpkg tool to install repositories and dependencies
ADD https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg /octool
RUN /octool -a -d

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
