#!/usr/bin/python

# SPCTools.py is a script containing various tools to help calibrating the
# parameters A and B of SizePenalty class

import sys
import math
from copy import copy

argv = sys.argv
argc = len(argv)

def sortDict(adict):
    keys = adict.keys()
    keys.sort()
    l=[]
    for k in keys:
        l.append((k, adict[k]))
    return l

def printDict(adict):
    l = sortDict(adict)
    for c in l:
        print c[0], c[1]
    return

def computeSizePenalty(a, b, doc, ioc, eoc, apc, aac, score, size):
    c = float(doc+ioc+eoc+apc+aac)
    res = math.exp(-a*math.log(b*c+math.exp(1.0))*float(size))
    print 'a =', a, ',b =', b
    print 'doc =', doc, ',ioc =', ioc, ',eoc =', eoc, ',apc =', apc, ',aac =', aac
    print 'score =', score, ',size =', size
    print 'c =', c, ',sizePenalty =', res
    return res

def computegs(a, b, fs):
    gs = []
    for f in fs:
        g = []
        condsActsPair = f[0]
        doc = condsActsPair[0]
        ioc = condsActsPair[1]
        eoc = condsActsPair[2]
        apc = condsActsPair[3]
        aac = condsActsPair[4]
        lp = f[1]
        for p in lp:
            score = p[0]
            size = p[1]
            sp = computeSizePenalty(a, b, doc, ioc, eoc, apc, aac, score, size)
            g.append((score, score*sp))
        gs.append(g)
    return gs

def isLastArgMax(l):
    argmax = 0
    maxval = 0.0
    argn = 0
    for p in l:
        if maxval<p[1]:
            maxval = p[1]
            argmax = argn
        argn += 1
    return argmax==(argn-1)

if argc==3:
    command = argv[1]
    data = open(argv[2]).read()
else:
    print 'SPCTools.py is a script containing various tools to help'
    print 'calibrating the parameters A and B of SizePenalty class'
    print 'Usage:'
    print 'SPCTools.py COMMAND FILE_NAME [OPTIONS]'
    print 'List of commands :'
    print '--------------------------------------'
    print '   e : takes a LS log and outputs on the standard output functions that maps the score to the size of the smallest program. Each function is represented with the following format :'
    print 'FUNCTION doc ioc eoc apc aac'
    print 'score_0_0 min_size_0_0'
    print 'score_0_1 min_size_0_2'
    print '...'
    print 'score_0_n min_size_0_n'
    print 'FUNCTION doc ioc eoc apc aac'
    print 'score_1_0 min_size_1_0'
    print '...'
    print 'score_1_m min_size_1_m'
    print
    print 'For 2 functions here with respectively n+1 and m+1 scores'
    print 'Where each argument after FUNCTION :'
    print 'doc stands for definite object count'
    print 'ioc stands for indefinite object count'
    print 'eoc stands for elementory operator count'
    print 'apc stands for atomic perception count'
    print 'aac stands for atomic action count'
    print '--------------------------------------'
    print '   c : takes a function (previously computed using the command e) and computes for a set parameter values whether they respect the constraint that the candidate with the best score must have the best fitness'
    exit(1)

if command=='e':
    
    dataList = data.split('\n')

    #initialize f, doc, ioc, eoc, apc and aac
    f={}
    doc = 0
    ioc = 0
    eoc = 0
    apc = 0
    aac = 0
    
    for l in dataList:
        wl = l.split()
        #determine the parameters of the new estimator
        if l.rfind('SPCTools - New estimator') != -1:
            #print 'new estimator'
            if len(f)!=0:
                printDict(f)
                f={}
        elif l.rfind('SPCTools - definite object count') != -1:
            #print 'definite object count'
            #print wl
            assert(len(wl)==12)
            doc = wl[11]
        elif l.rfind('SPCTools - indefinite object count') != -1:
            #print 'indefinite object count'
            #print wl
            assert(len(wl)==12)
            ioc = wl[11]
        elif l.rfind('SPCTools - elementary operator count') != -1:
            #print 'elementary operator count'
            #print wl
            assert(len(wl)==12)
            eoc = wl[11]
        elif l.rfind('SPCTools - atomic perception count') != -1:
            #print 'atomic perception count'
            #print wl
            assert(len(wl)==12)
            apc = wl[11]
        elif l.rfind('SPCTools - atomic action count') != -1:
            #print 'atomic action count'
            #print wl
            assert(len(wl)==12)
            aac = wl[11]
            print 'FUNCTION :', doc, ioc, eoc, apc, aac
        #determine the function maping
        elif l.rfind('SPCTools - Score') != -1:
            #print 'score'
            #print wl
            assert(len(wl)==10)
            score = float(wl[9])
        elif l.rfind('SPCTools - Combo size') != -1:
            #print 'combo size'
            #print wl
            assert(len(wl)==11)
            size = int(wl[10])
            if f.has_key(score):
                f[score] = min(f[score], size)
            else:
                f[score] = size

    printDict(f)
    

    exit(0)

if command=='c':

    #load functions
    dataList = data.split('\n')

    fs = []
    f = []
    
    for l in dataList:
        wl = l.split()
        if len(wl)==7:
            if wl[0]=='FUNCTION':
                if len(f)!=0:
                    fs.append(((doc, ioc, eoc, apc, aac),f))
                doc = float(wl[2])
                ioc = float(wl[3])
                eoc = float(wl[4])
                apc = float(wl[5])
                aac = float(wl[6])
                f = []
        elif len(wl)==2:
            f.append((float(wl[0]), float(wl[1])))

    fs.append(((doc, ioc, eoc, apc, aac),f))

    #determine the set of values to check
    ar = [ x/100.0 for x in range(2, 6) ]
    br = [ x/100.0 for x in range(0, 200) ]

    #evaluate for each parameter point if constraint is valide,
    #that is best score has best fitness
    for a in ar:
        for b in br:
            gs = computegs(a, b, fs)
            for g in gs:
                print 'a =', a, ',b =', b, ',is last arg max =', isLastArgMax(g)

    exit(0)

else:
    print 'Wrong command, run SPCTools.py without arguments to see the lists of command available'
    exit(1)
