#!/usr/bin/python
import sys
import csv
import matplotlib.pyplot as plt
import numpy as np
import time as t
import re

cont = True;

while cont:
    cont = False

    f = open("dump-av.data",'r')
    lines = csv.reader(f)

    line0 = lines.__next__()

    groups = int(line0[0])
    groupsize = int(line0[1])

    data = (np.zeros([groups+2,groupsize,2,0])).tolist()
    current_group = [[],[]]

    colors = ['r','g','b','k','c','m']


    for line in lines:
        data[groups+1][0][0].append(float(line[6]))
        data[groups+1][0][1].append(float(line[4]))
        current_group[0].append(float(line[5]))
        current_group[1].append(float(line[4]))
        if (line[0].startswith("group")):
            ints = [int(s) for s in re.findall(r'\d+', line[0])]
            group = ints[0]
            word = ints[1]
            data[group][word][0].append(float(line[1]))
            data[group][word][1].append(float(line[4]))
        elif (line[0].startswith("non")):
            word = int(line[0][10:])
            if word >= groupsize:
                continue
            data[groups][word][0].append(float(line[1]))
            data[groups][word][1].append(float(line[4]))

    f = open("dump-hebtv.data",'r')

    lines = csv.reader(f)

    hebtv = {'null': [[],[]]}

    for line in lines:
        #if line[1].startswith("group0"): #and line[2].startswith("group1"):
        #if line[1][:6] != line[2][:6]:
            try:
                if line[0] in hebtv:
                    hebtv[line[0]][0].append(float(line[3]))
                    hebtv[line[0]][1].append(float(line[5]))
                else:
                    hebtv[line[0]] = [[],[]]
                    hebtv[line[0]][0].append(float(line[3]))
                    hebtv[line[0]][1].append(float(line[5]))
            except:
                print(line)


    fig = plt.figure()

   #ax1 = fig.add_subplot(211)
   #ax1.plot(current_group[1],current_group[0],color='y')

    ax2 = fig.add_subplot(121)
    for i in range(groups+2):
        for w in range(groupsize):
           #if i == groups:
           #    continue
            clr = plt.cm.jet(1. * i / (groups+1))
            ax2.plot(data[i][w][1],data[i][w][0],color=clr)

    ax3 = fig.add_subplot(122)
    for key,e in hebtv.items():
        ax3.plot(e[1],e[0])

    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()
