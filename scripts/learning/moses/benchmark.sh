#!/bin/bash
#
# run several instances of MOSES with different options

# number of evaluations
m=100000
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
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf2
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf3
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf4
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf5
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf6
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf7
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf8
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf9
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf10
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf11
P=200
E=2
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf12
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf13
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf14
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf15
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf16
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf17
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf18
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf19
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf20
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf21
P=2000
E=2
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf22
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf23
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf24
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf25
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf26
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf27
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf28
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf29
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &

# conf30
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f -L > /dev/null &
