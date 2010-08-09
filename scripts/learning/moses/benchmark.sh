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

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf2
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf3
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf4
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf5
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf6
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf7
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf8
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf9
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf10
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf11
P=200
E=2
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf12
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf13
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf14
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf15
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf16
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf17
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf18
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf19
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf20
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf21
P=2000
E=2
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf22
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf23
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf24
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf25
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf26
E=3
r=1

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf27
r=12

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf28
r=123

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf29
r=1234

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &

# conf30
r=12345

./moses-exec -h pa -r "$r" -m "$m" -k "$k" -E "$E" -B "$B" -P "$P" -a "$a" -f "moses_${r}_${k}_${E}_${B}_${P}_${a}.log" > "moses_${r}_${k}_${E}_${B}_${P}_${a}.res" &
