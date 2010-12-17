#!/usr/bin/env python

# Requires matplotlib for graphing
# reads *_benchmark.csv files as output by atomspace_bm and turns them into
# graphs.


import csv
import numpy as np
import matplotlib.colors as colors
#import matplotlib.finance as finance
import matplotlib.dates as mdates
import matplotlib.ticker as mticker
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
#import matplotlib.font_manager as font_manager

import glob
import pdb


def moving_average(x, n, type='simple'):
    """
    compute an n period moving average.

    type is 'simple' | 'exponential'

    """
    x = np.asarray(x)
    if type=='simple':
        weights = np.ones(n)
    else:
        weights = np.exp(np.linspace(-1., 0., n))

    weights /= weights.sum()

    a =  np.convolve(x, weights, mode='full')[:len(x)]
    a[:n] = a[n]
    return a

def graph_file(fn,delta_rss=True):
    print "Graphing " + fn
    records = csv.reader(open(fn,'rb'),delimiter=",")
    sizes=[]; times=[]; times_seconds=[]; memories=[]
    for row in records:
        sizes.append(int(row[0]))
        times.append(int(row[1]))
        memories.append(int(row[2]))
        times_seconds.append(float(row[3]))

    left, width = 0.1, 0.8
    rect1 = [left, 0.5, width, 0.4]  #left, bottom, width, height
    rect2 = [left, 0.1, width, 0.4]

    fig = plt.figure(facecolor='white')
    axescolor  = '#f6f6f6'  # the axies background color

    ax1 = fig.add_axes(rect1, axisbg=axescolor)
    ax2 = fig.add_axes(rect2, axisbg=axescolor, sharex=ax1)

    ax1.plot(sizes,times_seconds,color='black')
    if len(times_seconds) > 1000:
        ax1.plot(sizes,moving_average(times_seconds,len(times_second) / 100),color='blue')
    if delta_rss:
        oldmemories = list(memories)
        for i in range(1,len(memories)): memories[i] = oldmemories[i] - oldmemories[i-1]
    ax2.plot(sizes,memories,color='black')

    for label in ax1.get_xticklabels():
        label.set_visible(False)

    class MyLocator(mticker.MaxNLocator):
        def __init__(self, *args, **kwargs):
            mticker.MaxNLocator.__init__(self, *args, **kwargs)

        def __call__(self, *args, **kwargs):
            return mticker.MaxNLocator.__call__(self, *args, **kwargs)

    # at most 7 ticks, pruning the upper and lower so they don't overlap
    # with other ticks
    fmt = mticker.ScalarFormatter()
    fmt.set_powerlimits((-3, 4))
    ax1.yaxis.set_major_formatter(fmt)

    ax2.yaxis.set_major_locator(MyLocator(7, prune='upper'))
    fmt = mticker.ScalarFormatter()
    fmt.set_powerlimits((-3, 4))
    ax2.yaxis.set_major_formatter(fmt)
    ax2.yaxis.offsetText.set_visible(False)
    fig.show()
    size = int(fmt.orderOfMagnitude) / 3
    labels = ["B","KB","MB","GB"]
    label = labels[size]
    labels = ["","(10s)","(100s)"]
    label += " " + labels[int(fmt.orderOfMagnitude) % 3]

    ax2.set_xlabel("AtomSpace Size")
    ax2.set_ylabel("RSS " + label)
    ax1.set_ylabel("Time (seconds)")
    ax1.set_title(fn)
    fig.show()

    fig.savefig(fn+".png",format="png")

files_to_graph = glob.glob("*_benchmark.csv")

for fn in files_to_graph:
    graph_file(fn);




