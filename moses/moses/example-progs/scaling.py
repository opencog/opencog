#!/usr/bin/env python
#
# Very simple script to perform timing measurements on disributed moses.
# Print out the speedup, the parallelizable fraction, and the max speedup.
#
import os
import sys
import time
from datetime import datetime

def toflt(delta):
    return float(delta.seconds) + 0.000001 * float(delta.microseconds)

num_to_do = 6

for nodes in range(2,12):

    before = datetime.now()
    tot_time = before - before

    for rseed in range (1,1+num_to_do) :
        cmd = "mpirun -n " + str(nodes) + " --hostfile ~/mpd.hosts ./moses/moses/main/moses -Hit -Y1 -u2 -i ./wdbc.data -W1 -x1 -n sin -n log -n exp -Z1 -v12 --hc-max-nn-evals=5000 -r" + str(rseed) + " -j12 -m320000 --mpi=1 -fmspeed.log"
        print cmd
        # measure wallcock time.
        before = datetime.now()
        os.system(cmd)
        after = datetime.now()
        elapsed = after - before
        print "seed=", rseed, " elapsed wallclock time: ", elapsed
        tot_time += elapsed
        sys.stdout.flush()
        time.sleep(15)

    elapsed = tot_time
    bsecs = toflt(elapsed) / num_to_do

    if nodes == 2:
        baseline = elapsed
        print "Baseline time: ", baseline, " seconds=", bsecs

    if nodes != 2:
        speedup = toflt(baseline) / toflt(elapsed)
        parallel = (1.0 - 1.0 / speedup) / (1.0 - 1.0 / float(nodes-1))
        maxspeed = 1.0 / (1.0 - parallel)
        print "workers=", nodes-1, " elapsed=", bsecs, " speedup=", speedup, " parallelizable fraction=", parallel, " max speedup=", maxspeed

    sys.stdout.flush()
    time.sleep(15)


