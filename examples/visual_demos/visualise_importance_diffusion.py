#!/usr/bin/python2.5

import sys
import os
import subprocess
import time

# the visualiser executable, make blank if you don't want it to be spawned
# automatically
ubigraph_server="/home/joel/src/UbiGraph-alpha-0.2.4-Linux64-Ubuntu-8.04/bin/ubigraph_server"

N = 10

def main():
    #/sys.argv()

    #print "Spawning ubigraph server",
    #pid = subprocess.Popen([ubigraph_server])
    #if pid.returncode is not None:
    #    print ".. failed"
    #    sys.exit(1)
    #print ".. started"

    printHeader()
    makeNet(30)
    printTail()

def makeNet(N):

    for i in range(0,N):
        jN = 1
        if i == N / 2:
            jN = N
        for j in range(0,jN):
            node_name = str(i) + "__" + str(j)
            print "(cog-new-node 'ConceptNode \"" + node_name + "\")"
            # connect to above row
            if i > 0 and j == 0:
                makeLink(str(i-1) + "__" + str(j),node_name)
            # connect to previous column's node
            if j > 0:
                makeLink(str(i) + "__" + str(j-1),node_name)

def makeLink(a,b):
    print "(cog-new-link 'SymmetricHebbianLink " + \
        "(cog-node 'ConceptNode \"" + a + "\") " + \
        "(cog-node 'ConceptNode \"" + b + "\") " + \
        "(cog-new-stv 0.05 0.8))"

def printHeader():

    print """loadmodule opencog/ubigraph/libubigraph.so
ubigraph --compact
loadmodule opencog/dynamics/attention/libattention.so
agents-stop-loop
agents-start opencog::ImportanceDiffusionAgent
"""
    sys.stdout.flush()
    print "scm"

def printTail():

    print "." # close scm shell

    sys.stdout.flush()
    time.sleep(10)
    for i in range(0,100):
        if i % 10 == 0:
            print "ubigraph-random-sti 5"
            print "ubigraph-update-sti"
            
        print """agents-step
ubigraph-update-sti
        """
        sys.stdout.flush()
        time.sleep(1)


if __name__ == '__main__':
     main()
