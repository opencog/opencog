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

rest_url = 'http://localhost:17034/rest/0.2/'

def spawn_server():
    global server_process
    server_process = subprocess.Popen([server_exe]) #, '-DLOG_TO_STDOUT=TRUE'])
    time.sleep(1) # Allow modules time to load
    print "Server spawned with pid %d" % (server_process.pid,)

def kill_server():
    server_process.kill()
    print "Server killed"

class TestPostAtom(unittest.TestCase):
    
#def setUp(self):
        #pass

    #def tearDown(self):
        #pass
    
    def testPostSuccess(self):
        data = '{ "type":"ConceptNode", "name":"a test for the times", "truthvalue": {"simple": {"str":0.5, "count":10}}}'

#data = urllib.urlencode(values)
#        print data
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

if __name__ == "__main__":
    spawn_server()
    del sys.argv[1]
    unittest.main()
    kill_server()
    sys.exit(0)

