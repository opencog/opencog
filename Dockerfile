# For use with Docker, a manager for Linux Containers
# Download from https://www.docker.io/gettingstarted/
# Build: sudo docker build -t $USER/opencog .
# Run: CONTAINER_ID=$(sudo docker run -d -p 17001:17001 -p 5000:5000 $USER/opencog)
# Find IP: CONTAINER_IP=$(sudo docker inspect $CONTAINER_ID | grep IPAddress | cut -d '"' -f 4)
# Find password: sudo docker logs $CONTAINER_ID | sed -n 1p
# Login: ssh docker@$CONTAINER_IP
# Adjust apt sources to match your local mirror

FROM ubuntu:12.04
MAINTAINER David Hart "dhart@opencog.org"

ENV DEBIAN_FRONTEND noninteractive
RUN echo "deb http://hk.archive.ubuntu.com/ubuntu precise main universe" > /etc/apt/sources.list && \
    echo "deb http://hk.archive.ubuntu.com/ubuntu precise-updates main universe" >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get -y install python-software-properties wget ssh sudo pwgen

# Work around Upstart/DBus issues & set locale (fix the locale warnings)
RUN dpkg-divert --local --rename --add /sbin/initctl && \
    ln -v -s /bin/true /sbin/initctl && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo dpkg-reconfigure locales
#   localedef -c -i en_US -f UTF-8 en_US.UTF-8 || :

# Get ocpkg script, replace wget with ADD when caching feature is added
RUN mkdir -v /opencog && \
    wget -N https://raw.github.com/githart/opencog-config-files/master/docker/opencog/precise/ocpkg -O /opencog/ocpkg && \
    chmod u+x /opencog/ocpkg

# Run ocpkg/octool to: add repositories, install depencies, update source, build OpenCog
RUN ln -s -v /opencog/ocpkg /opencog/octool
RUN /opencog/octool -v -a
RUN /opencog/octool -v -d
RUN /opencog/octool -v -u
RUN mkdir -v -p /opencog/src/bin
RUN /opencog/octool -v -b -l /opencog/src/bin

# Copy context files into the container
ADD . /opencog

EXPOSE 22
CMD ["/bin/bash", "/opencog/docker-container-init.sh"]
