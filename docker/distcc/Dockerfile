# For use with Docker https://www.docker.io/gettingstarted/
#
# docker build -t $USER/opencog-distcc .
# docker run -d -p 3632:3632 $USER/opencog-distcc

FROM opencog/opencog-deps
MAINTAINER David Hart "dhart@opencog.org"

RUN apt-get -y update && \
    apt-get -y install distcc

# startdtiscc ignored since we're manually starting in non-daemon mode
RUN sed -i s:STARTDISTCC="false":STARTDISTCC="true":g /etc/default/distcc 
RUN sed -i s:ALLOWEDNETS="127.0.0.1/24":ALLOWEDNETS="158.132.58.0/24":g /etc/default/distcc 
RUN sed -i s:LISTENER="127.0.0.1":LISTENER="0.0.0.0":g /etc/default/distcc 
RUN sed -i s:NICE="10":NICE="10":g /etc/default/distcc 
RUN sed -i s:JOBS="":JOBS="8":g /etc/default/distcc 
RUN sed -i s:ZEROCONF="false":ZEROCONF="true":g /etc/default/distcc 

CMD /usr/bin/distccd --allow 158.132.58.0/24 ; sleep infinity
