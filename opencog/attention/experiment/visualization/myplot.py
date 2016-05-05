#!/usr/bin/python
import sys
import csv
import matplotlib.pyplot as plt
import numpy as np

data = [[[],[]]] * 4

colors = ['r','g','b','k','c','m','y']

f = open("dump-av.data",'r')
lines = csv.reader(f);
for line in lines:
    if not(line[0].startswith("group")):
        continue
    group = int(line[0][5])
    data[group][0].append(float(line[1]))
    data[group][1].append(float(line[4]))

i = 0

fig = plt.figure()

for group in data:
    ax = fig.add_subplot(220+i+1)
    ax.plot(group[1],group[0],color=colors[i])
    i = i + 1

plt.show()

