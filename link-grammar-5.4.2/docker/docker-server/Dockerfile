#
# Dockerfile for the link-grammar parse server.
#
# The parse server is currently written so that it requires the Java
# bindings to run. So this has to pull down thw whole fatso JDK.
#
# XXX TODO -- actually start and run the parse server ... 
# ... and make sure it works.
#
FROM linkgrammar/lgbase:latest
MAINTAINER Linas Vepstas linasvepstas@gmail.com

# The parse server needs jni.h -- this is furnished by jdk
# which in turn has a HUGE dependency list :-(
# This is the reason we are doing multiple docker files, and not one.
RUN apt-get install -y openjdk-7-jdk
RUN apt-get install -y ant

# Perform the standard build.
ENV JAVA_HOME /usr/lib/jvm/java-7-openjdk-amd64
RUN (cd link-grammar-5*; mkdir build; cd build; ../configure; make -j12; make install; ldconfig)

# The link-parser server runs on port 9000 by default
EXPOSE 9000

RUN adduser --disabled-password --gecos "Link Parser User" link-parser

USER link-parser
WORKDIR /home/link-parser
RUN echo "export LANG=en_US.UTF-8" >> .bash_aliases
CMD bash

# The English-language server, for now ...
RUN export LANG=en_US.UTF-8

RUN echo "this is a test" | link-parser
# 
RUN cp /link-grammar-5*/bindings/java/link-grammar-server.sh .

# ./link-grammar-server.sh

