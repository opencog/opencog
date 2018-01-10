#
# Dockerfile for the link-grammar parser.
# Allows one to just build and compile the command-line tool,
# and so perform general experimentation.
#
FROM linkgrammar/lgbase:latest
MAINTAINER Linas Vepstas linasvepstas@gmail.com

# Build the libraries and command-line parser only
# Assumes that the sources have already been unpacked.
RUN (cd link-grammar-5*; mkdir build; cd build; ../configure --disable-java-bindings; make -j12; make install; ldconfig)

RUN adduser --disabled-password --gecos "Link Parser User" link-parser

USER link-parser
WORKDIR /home/link-parser
RUN echo "export LANG=en_US.UTF-8" >> .bash_aliases
CMD bash

RUN export LANG=en_US.UTF-8

RUN echo "this is a test" | link-parser
# 

