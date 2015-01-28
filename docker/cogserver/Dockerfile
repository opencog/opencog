# docker build -t $USER/cogserver .

FROM opencog/opencog-deps

# Clone opencog and build it
RUN git clone --depth 1 https://github.com/opencog/opencog oc
RUN (cd oc; mkdir build; cd build; cmake ..; make -j6)

WORKDIR /home/opencog/
USER opencog

# Docker defaults
## Start cogserver when container runs
ENTRYPOINT ["/home/opencog/oc/build/opencog/server/cogserver"]
CMD ["-c",  "oc/lib/opencog.conf"]
