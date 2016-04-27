#!/usr/bin/python
import sys
import csv
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.interpolate import spline


# data {size => [AnnaCycle,BobCycle,EdwardCycle,FrankCycle]}
#data = {}
xval = []
yval = []

def plot2():
    # Create a canvas to place the subgraphs
    canvas = plt.figure()
    rect = canvas.patch
    rect.set_facecolor('white')

    x_sm = np.array(xval).astype(np.float)
    print x_sm
    y_sm = np.array(yval[0]).astype(np.float)
    print y_sm
    x_smooth = np.linspace(x_sm.min(), x_sm.max(), 200)
    y_smooth1 = spline(np.array(xval,dtype=float), np.array(yval[0],dtype=float), x_smooth)
    y_smooth2 = spline(np.array(xval,dtype=float), np.array(yval[1],dtype=float), x_smooth)
    y_smooth3 = spline(np.array(xval,dtype=float), np.array(yval[2],dtype=float), x_smooth)
    y_smooth4 = spline(np.array(xval,dtype=float), np.array(yval[3],dtype=float), x_smooth)
    # Placing the plot1 on 1x1 matrix, at pos 1
    sp1 = canvas.add_subplot(1,1,1, axisbg='w')
    #sp1.plot(x, y, 'red', linewidth=2)
    sp1.plot(x_smooth, y_smooth1, 'r', linewidth=1)
    sp1.plot(x_smooth, y_smooth2, 'b', linewidth=1)
    sp1.plot(x_smooth, y_smooth3, 'g', linewidth=1)
    sp1.plot(x_smooth, y_smooth4, 'y', linewidth=1)
    # Colorcode the tick tabs 
    sp1.tick_params(axis='x', colors='gray')
    sp1.tick_params(axis='y', colors='gray')

    # Colorcode the spine of the graph
    sp1.spines['bottom'].set_color('black')
    sp1.spines['top'].set_color('black')
    sp1.spines['left'].set_color('black')
    sp1.spines['right'].set_color('black')

    # Put the title and labels
    sp1.set_title('ECAN guided smokes inference', color='black')
    sp1.set_xlabel('Noise data size', color='black')
    sp1.set_ylabel('Congnitive cycle', color='black')

    # Show the plot/image
    plt.tight_layout()
    plt.grid(alpha=0.8)
    plt.savefig("example6.eps")
    plt.show()
def plot():
  print "get val\n"
  print xval
  p= plt.plot(xval,yval[0],marker=".",label="Anna")
  plt.plot(xval,yval[1],marker=".",label="Bob")
  plt.plot(xval,yval[2],marker=".",label="Edward")
  plt.plot(xval,yval[3],marker=".",label="Frank")
  xlabel = "Total noise Size"
  ylabel = "Cycles taken"
  plt.xlabel(xlabel)
  plt.ylabel(ylabel)
  plt.legend()
  plt.show()


def set_data():
  global xval
  global yval
  files = ["Anna.data","Bob.data","Edward.data","Frank.data"]
  first = True
  for fl in files:
      f = open(fl,'r')
      yv =[]
      lines = csv.reader(f)
      for line in lines:
          if first:
              xval.append(line[0])
          yv.append(line[1])
      yval.append(yv)
      first = False



if __name__ == "__main__":
  set_data()
  plot2()


