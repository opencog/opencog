# For creating images with non gui tools for development and dependencies installed
# Steps:
# 1. docker build -t $USER/opencog-dev:cli .
# 2. docker create --name opencog -p 17001:17001 -p 5000:5000
#       -v ABSOLUTE/PATH/TO/OPENCOG/ROOT/DIRECTORY:/home/opencog
#       -it $USER/opencog-dev:cli
# 3. docker start -i opencog
# 4. mkdir build
# 5. cmake ..
# 6. make -j$(nproc)
# Check http://wiki.opencog.org/w/Building_OpenCog on how to use IDEs

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get update --assume-yes

RUN apt-get install \
                    byobu \
                    less \
                    curl \
                    mosh \
                    vim \
                    pv \
                    bc \
    --assume-yes

USER opencog
CMD /bin/bash
