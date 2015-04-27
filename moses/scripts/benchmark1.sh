#!/bin/bash
#
# run several instances of MOSES with different options

# number of evaluations
m=200000
# random seed
r=1
# size of the problem
k=3
# reduct effort for candidates
E=2
# reduct effort for knob building
B=2
# population size ratio
P=20
# algorithm
a=hc

# conf1
r=11

moses-exec -H pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -L > /dev/null &

# conf2
r=22

moses-exec -H pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -L > /dev/null &

# conf3
E=3
r=11

moses-exec -H pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -L > /dev/null &

# conf4
r=22

moses-exec -H pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -L > /dev/null &
