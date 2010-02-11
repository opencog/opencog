#!/usr/bin/env python
# Test REST interface by spawning server and sending a bunch of commands.

import sys
import os
import time
import subprocess
import unittest
import json

import urllib2
import urllib

server_exe = sys.argv[1]
server_process=None

base_url = 'http://localhost:17034/'
rest_url = base_url + 'rest/0.2/'

def spawn_server():
    global server_process
    print "Spawning server"
    server_process = subprocess.Popen([server_exe, '-c',
            '../../../lib/opencog.conf'])
#stderr=subprocess.PIPE, stdout=subprocess.PIPE) #, '-DLOG_TO_STDOUT=TRUE'])
    time.sleep(1) # Allow modules time to load
    print "Server spawned with pid %d" % (server_process.pid,)

def kill_server():
    print "Killing server"
    req = urllib2.Request(base_url + '/opencog/request/shutdown')
    response = urllib2.urlopen(req).read()
    stdout,stderr = server_process.communicate()
    print "Server killed"

class TestPostAtom(unittest.TestCase):
    
    def setUp(self):
        spawn_server()

    def tearDown(self):
        kill_server()
    
    def testPostSuccess(self):
        data = '{ "type":"ConceptNode", "name":"a test for the times", "truthvalue": {"simple": {"str":0.5, "count":10}}}'
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

    def testPostMerge(self):
        data = '{ "type":"ConceptNode", "name":"a test for the times1", "truthvalue": {"simple": {"str":0.5, "count":10}}}'
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "merged")

if __name__ == "__main__":
    del sys.argv[1]
    print "Starting REST interface test"
    unittest.main()

