# Replace all instances of shujingke with your github / docker.io / system username.
# Replace hk.archive.ubuntu.com with your own country code, e.g. nl.archive.ubuntu.com
# adduser shujingke
# cd ~ && git clone http://shujingke@github.com/shujingke/opencog && cd opencog && git pull
# docker build -t shujingke/opencog-dev-qt .
# xhost +
# docker run --rm -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /home/shujingke:/home/shujingke -e DISPLAY=:0.0 -t shujingke/opencog-dev-qt 

# New run command:
# docker run --rm -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -v /home/shujingke:/home/shujingke -e DISPLAY=:0.0 -p 17001:17001 -t shujingke/opencog-dev-qt

# Newer run command:
# ssh -X hostname 'docker run --rm -i -v /tmp/.X11-unix/X0:/tmp/.X11-unix/X0 -v /dev/dri:/dev/dri -v /dev/shm:/dev/shm -v /home/shujingke:/home/shujingke -e DISPLAY=$DISPLAY -p 17001:17001 -t shujingke/opencog-dev-qt'

FROM ubuntu:14.04
MAINTAINER Alex van der Peet "alex.van.der.peet@gmail.com"
MAINTAINER David Hart "dhart@opencog.org"

RUN sed 's/archive.ubuntu.com/hk.archive.ubuntu.com/' -i /etc/apt/sources.list

RUN apt-get -y update
RUN apt-get -y install software-properties-common git

ADD scripts/ocpkg install-dependencies-trusty
RUN chmod +x /install-dependencies-trusty
RUN /install-dependencies-trusty

RUN apt-get -y install wget tmux
RUN apt-get -y install gitg
RUN apt-get -y install git-gui
RUN apt-get -y install meld
RUN apt-get -y install qtcreator
RUN apt-get -y install gnome-session
RUN apt-get -y install gnome-panel
RUN apt-get -y install gnome-terminal
RUN apt-get -y install nautilus
RUN apt-get -y install vim-gnome

RUN adduser --disabled-password --gecos "Shujing Ke,,," shujingke

WORKDIR /home/shujingke
USER shujingke
ENV STARTSCRIPT "\
echo evaluating startup script... ;\
tmux new-session -d '/usr/bin/gnome-panel&/bin/bash' ;\
tmux set -g set-remain-on-exit on ;\
tmux set-option -g set-remain-on-exit on ;\
tmux bind-key R respawn-window ;\
tmux split-window -d -v -p 25 '/bin/bash' ;\
tmux select-layout even-vertical ;\
tmux attach \
"

CMD /bin/bash -l -c "eval $STARTSCRIPT"
