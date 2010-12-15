#!/usr/bin/env python

import csv
import numpy as np
import matplotlib.colors as colors
#import matplotlib.finance as finance
import matplotlib.dates as mdates
import matplotlib.ticker as mticker
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
#import matplotlib.font_manager as font_manager

records = csv.reader(open('bm_addLink.csv','rb'),delimiter=",")
sizes=[]; times=[]; memories=[]
for row in records:
    sizes.append(int(row[0]))
    times.append(int(row[1]))
    memories.append(int(row[2]))

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

CLOCKS_PER_SEC=1000000
times_seconds = [ float(x) / CLOCKS_PER_SEC for x in times ]

left, width = 0.1, 0.8
rect1 = [left, 0.5, width, 0.4]
rect2 = [left, 0.1, width, 0.4]

fig = plt.figure(facecolor='white')
axescolor  = '#f6f6f6'  # the axies background color

ax1 = fig.add_axes(rect1, axisbg=axescolor)  #left, bottom, width, height
ax2 = fig.add_axes(rect2, axisbg=axescolor, sharex=ax1)

ax1.set_xlim(180000,180500)
ax1.plot(sizes,times_seconds,color='black')
ax1.plot(sizes,moving_average(times_seconds,1000),color='blue')
ax2.set_xlim(180000,180500)
ax2.plot(sizes,memories,color='black')



