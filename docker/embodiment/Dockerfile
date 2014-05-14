# For use with Docker https://www.docker.io/gettingstarted/
#
# Quickstart:
# docker build -t $USER/opencog-embodiment .
# docker run --name embodiment -d -p 17001:17001 -p 5000:5000 -p 16312:16312 $USER/opencog-embodiment
# docker logs embodiment
 
FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"
 
# Set locale
RUN sudo locale-gen en_US en_US.UTF-8 && \
    sudo dpkg-reconfigure locales
 
# Get ocpkg script, replace wget with ADD when caching feature is added
ADD . /opencog
RUN  mkdir -v -p /opencog/build 
 
# Run special scripts to copy embodiment files (should be in cmake not tclsh!)
#RUN apt-get -y install lsof psmisc

WORKDIR /opencog/scripts/embodiment
RUN ./make_distribution
 
# Start embodiment cogserver, spawner, router, etc. when container runs
WORKDIR /opencog/build/dist/embodiment
CMD ["/opencog/build/dist/embodiment/spawner"]
