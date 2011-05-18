#!/usr/bin/env python
#
# Publish mock messages via ZeroMQ
# 
# It randomly generate some values to fool the boss which makes nonsense 
# 
# @author Zhenhua Cai <czhedu@gmail.com>
# @date   2011-04-22
#
# @note   This server is only used for development 

import time
import zmq
import json
import random

context = zmq.Context()

oac_socket = context.socket(zmq.PUB)
oac_socket.bind("tcp://*:18002")

relex_server_socket = context.socket(zmq.PUB)
relex_server_socket.bind("tcp://*:16316")

timestamp = random.randrange(100000000, 200000000)
#timestamp = 100000000

###############################################################################
#
def publish_modulator_message(timestamp):
    # Publish the key
    key = "PsiModulatorUpdaterAgent"
    oac_socket.send(key, zmq.SNDMORE) 

    # Randomly generate some values to fool the boss
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

    oac_socket.send(jsonObj)

    # Sleep for a random time
    time.sleep(random.random()/10)

###############################################################################
#
def publish_demand_message(timestamp):
    # Publish the key
    key = "PsiDemandUpdaterAgent"
    oac_socket.send(key, zmq.SNDMORE) 

    # Randomly generate some values to fool the boss
    energy = random.random()
    water = random.random()
    integrity = random.random()
    certainly = random.random()
    competence = random.random()
    affiliation = random.random()
    
    # Pack in json format and publish it via zeromq
    jsonObj = json.dumps( { 'timestamp': timestamp, 
                            'Energy': energy, 
                            'Water': water, 
                            'Integrity': integrity, 
                            'Certainly': certainly, 
                            'Competence': certainly, 
                            'Affiliation': affiliation
                          }
                        )

    oac_socket.send(jsonObj)

    # Sleep for a random time
    time.sleep(random.random()/10)

def publish_feeling_message(timestamp):
    # Publish the key
    key = "PsiFeelingUpdaterAgent"
    oac_socket.send(key, zmq.SNDMORE) 

    # Randomly generate some values to fool the boss
    happy = random.random()
    angry = random.random()
    love = random.random()
    hate = random.random()
    
    # Pack in json format and publish it via zeromq
    jsonObj = json.dumps( { 'timestamp': timestamp, 
                            'Happy': happy, 
                            'Angry': angry, 
                            'Love': love, 
                            'Hate': hate 
                          }
                        )

    oac_socket.send(jsonObj)

    # Sleep for a random time
    time.sleep(random.random()/10)

###############################################################################
#
def publish_relex_server_message():
    # Publish the key
    key = "RelexServer"
    relex_server_socket.send(key, zmq.SNDMORE) 

    # Randomly generate some values to fool the boss
    original_sentence = 'my name is le lei i am han mei none sense just to fool the boss'  
    sender_agent_id = random.randrange(1000, 9999)
    receiver_agent_id = random.randrange(1000, 9999)
    content_type  = "ContentType"
    target_mode = "TargetMode"
    
    # Pack in json format and publish it via zeromq
    jsonObj = json.dumps( { 'original_sentence': original_sentence, 
                            'sender_agent_id': sender_agent_id, 
                            'receiver_agent_id': receiver_agent_id, 
                            'content_type': content_type, 
                            'target_mode': target_mode 
                          }
                        )

    relex_server_socket.send(jsonObj)

    # Sleep for a random time
    time.sleep(random.random()/10)

###############################################################################
#
# Enter the main loop and publish messages forever
#
while True:
    # Increase the timestamp
    timestamp = timestamp + 1

    publish_modulator_message(timestamp)
    publish_demand_message(timestamp)
    publish_feeling_message(timestamp)

    publish_relex_server_message()

    time.sleep(random.random()*2)

