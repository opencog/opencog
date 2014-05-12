# For use with Docker https://www.docker.io/gettingstarted/
#
# Manual interactive buildsteps:
# ~/opencog/docker/bindmounts.sh
# docker build -t $USER/opencog-build .
# docker run -i -p 17001:17001 -t $USER/opencog-build /bin/bash
# cd /opencog
# mkdir build
# cmake ..
# make -j$(nproc)

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

ADD opencog /opencog

CMD /bin/bash
