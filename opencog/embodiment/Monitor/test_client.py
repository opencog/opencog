#!/usr/bin/env python
import sys

import zmq
import json

import numpy as np
import matplotlib.pyplot as plt

context = zmq.Context()
socket = context.socket(zmq.SUB)

print "Connecting to PsiModulatorUpdaterAgent ..."
socket.connect ("tcp://127.0.0.1:18002")

filter = sys.argv[1] if len(sys.argv) > 1 else "PsiModulatorUpdaterAgent"
#filter = ""
socket.setsockopt(zmq.SUBSCRIBE, filter)

#x = np.arange(0, 5, 0.1)
#print x
#y = np.sin(x)
#plt.plot(x, y)
#plt.show()

x_time = []
y_activation = []
y_resolution = []

#while True:
for i in range(1, 20):
    message = socket.recv()
    if message == filter: continue

    data = json.loads(message)
#    print string
#    print data['timestamp']
#    print data['Activation']
#    print data['Resolution']

    x_time.append(data['timestamp'])
    y_activation.append(data['Activation'])
    y_resolution.append(data['Resolution'])


plt.plot(x_time, y_activation)
plt.plot(x_time, y_resolution)

plt.show()


