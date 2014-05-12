# For use with Docker https://www.docker.io/gettingstarted/
#
# docker build -t $USER/opencog-buildslave .
# docker run -d -e BUILDSLAVE_PASSWD=foobar $USER/opencog-buildslave

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

#RUN apt-get install -f buildbot-slave

RUN apt-get -y update && \
    apt-get -y install python-pip python-dev

RUN pip install buildbot-slave==0.8.8

RUN adduser --disabled-password -uid 1099 --gecos "Buildbot,,," --home /buildbot buildbot

RUN su buildbot sh -c "buildslave create-slave --umask=022 /buildbot buildbot.opencog.org:9989 sanfeng-slave os.getenv\(\'BUILDSLAVE_PASSWD\'\)"

# remove single quotes from around passwd string passed to create-slave
RUN sed -i s/.\'os.\getenv/os\.getenv/g /buildbot/buildbot.tac && \
    sed -i s/PASSWD\'\)\'/PASSWD\'\)/g /buildbot/buildbot.tac

# set buildslave admin and host info
RUN sed -i "s/Your Name Here <admin@youraddress.invalid>/David Hart <dhart@opencog.org>/g" /buildbot/info/admin && \
    echo "Intel Core i7-3770 CPU @ 3.40GHz 16GB RAM\nUbuntu 14.04 (Trusty Tahr)" > /buildbot/info/host

CMD su buildbot sh -c "/usr/local/bin/buildslave start --nodaemon /buildbot"
