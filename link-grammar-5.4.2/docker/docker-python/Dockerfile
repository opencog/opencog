#
# Experimental dockerfile for link-grammar python bindings.
#
# XXX TODO: actually run the python unit tests!
#
FROM linkgrammar/lgbase:latest
MAINTAINER Linas Vepstas linasvepstas@gmail.com

RUN apt-get install -y swig2.0
RUN apt-get install -y python
RUN apt-get install -y python-dev
RUN apt-get install -y zlib1g-dev

# Perform the standard build.
RUN (cd link-grammar-5*; mkdir build; cd build; ../configure --disable-java-bindings --enable-python-bindings; make -j12; make install; ldconfig)

RUN adduser --disabled-password --gecos "Link Parser User" link-parser

USER link-parser
WORKDIR /home/link-parser
RUN echo "export LANG=en_US.UTF-8" >> .bash_aliases
CMD bash

RUN export LANG=en_US.UTF-8

RUN echo "this is a test" | link-parser

