# docker build -t $USER/opencog-dev .
# docker run -i -p 17001:17001 -v /home:/home -t $USER/opencog-dev /usr/bin/byobu
# cd /opencog
# mkdir build
# cmake ..
# make -j$(nproc)

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get update --assume-yes

RUN apt-get install \
                    byobu \
                    less \
                    curl \
                    wget \
                    mosh \
                    vim \
                    git \
                    pv \
                    bc \
    --assume-yes

CMD /bin/bash
