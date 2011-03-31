import time
import zmq
import json
import random

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:18002")

#timestamp = random.randrange(100000000, 200000000)
timestamp = 100000000

while True:
    # Publish the key
    key = "PsiModulatorUpdaterAgent"
    socket.send(key, zmq.SNDMORE) 

    # Randomly generate some values to fool the boss
    timestamp = timestamp + 1
    activation = random.random()
    resolution = random.random()
    securing_threshold = random.random()
    selection_threshold = random.random()
    
    # Pack in json format and publish it via zeromq
    jsonObj = json.dumps( { 'timestamp': timestamp, 
                            'Activation': activation, 
                            'Resolution': resolution, 
                            'SecuringThreshold': securing_threshold, 
                            'SelectionThreshold': selection_threshold
                          }
                        )

    socket.send(jsonObj)

    time.sleep(0.4)

