#!/usr/bin/env python
# Test REST interface by spawning server and sending a bunch of commands.

import sys
import os
import time
import subprocess
import unittest
import json
import pdb

import urllib2
import urllib

server_exe = sys.argv[1]
if len(sys.argv) > 2: opencog_conf = sys.argv[2]
else: 
    opencog_conf = '../lib/opencog-test.conf'
if not os.path.isfile(opencog_conf):
    opencog_conf = '../lib/opencog.conf'
del sys.argv[1:3]
server_process=None

base_url = 'http://localhost:17034/'
rest_url = base_url + 'rest/0.2/'
connected = True

def spawn_server():
    global server_process
    print "Spawning server"
    server_process = subprocess.Popen([server_exe, '-c',
            opencog_conf], stdout=sys.stdout)
    # while not connected: 
        # try:
            # req = urllib2.Request(rest_url + 'atom/',data)
            # response = urllib2.urlopen(req).read()
        # except:
    # TODO add a CogServer command to see if all preload modules have been setup

    time.sleep(10) # Allow modules time to load
    print "Server spawned with pid %d" % (server_process.pid,)

def kill_server():
    print "Killing server"
    server_process.terminate()
    #req = urllib2.Request(base_url + '/opencog/request/shutdown')
    #response = urllib2.urlopen(req).read()
    #stdout,stderr = server_process.communicate()
    print "Server killed"

class TestPostAtom(unittest.TestCase):

    def setUp(self):
        pass

    def tearDown(self):
        pass
        
    def testPostSuccess(self):
        """ Just test a very basic add """
        data = '{ "type":"ConceptNode", "name":"testPostSuccess1", "truthvalue": {"simple": {"str":0.5, "count":10}}}'
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")
        h1 = result["handle"]

        data = '{ "type":"ConceptNode", "name":"testPostSuccess2", "truthvalue": {"simple": {"str":0.5, "count":10}}}'
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")
        h2 = result["handle"]

        # Test creating a link
        data = { "type":"InheritanceLink",
            "outgoing": [h1,h2],
            "truthvalue": {"simple": {"str":0.5,
                "count":10}}}
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

    def testPostMerge(self):
        """ Test that merging happens if the atom already exists """
        data = '{ "type":"ConceptNode", "name":"testPostMerge", "truthvalue": {"simple": {"str":0.5, "count":10}}}'
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        req = urllib2.Request(rest_url + 'atom/',data)
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "merged")

    def testPostTVTypes(self):
        """ Test that all the different TV types parse correctly """
        data = { "type":"ConceptNode",
            "name":"SimpleTV",
            "truthvalue":
            {"simple": {"str":0.5, "count":10}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"CountTV",
            "truthvalue":
            {"count": {"str":0.5, "count":10, "conf":0.5}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"IndefiniteTV",
            "truthvalue":
            {"indefinite": {"l":0.5, "u":0.7, "conf":0.2}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"CompositeTV",
            "truthvalue":
            {"composite": {"primary": { "simple": {"str":0.5, "count":10}},
                "CONTEXTUAL":[1, {"simple": {"str":0.5, "count":10}}] }
            }
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

    def long_testPostTVRobustness(self):
        """ Test that bad json TV syntax fails gracefully, i.e. doesn't kill the
            server!
            Disable by default because it takes ~40 seconds to run
        """
        data = """{ "type":"ConceptNode",
            "name":"SimpleTVMutations",
            "truthvalue":
            {"simple": {"str":0.5, "count":10}}
        }"""
        import random
        import datetime
        print datetime.datetime.now()
        count=0
        r = random.Random()
        for i in range(0,len(data)):
            count+=1
            data_copy = list(data)
            data_copy[i] = ' '
            data_copy = ''.join(data_copy)
            req = urllib2.Request(rest_url + 'atom/',data_copy)
            response = urllib2.urlopen(req).read()
            try:
                result = json.loads(response)
            except ValueError:
                print '---'
                print response
            self.assertEqual(server_process.returncode, None)

        data =""" { "type":"ConceptNode",
            "name":"CompositeTVMutations",
            "truthvalue":
            {"composite": {"primary": { "simple": {"str":0.5, "count":10}},
                "CONTEXTUAL":[1, {"simple": {"str":0.5, "count":10}}] }
                "HYPOTHETICAL":[2, {"simple": {"str":0.5, "count":10}}] }
            }
        }"""
        for i in range(data.find("composite"),len(data)):
            count+=1
            data_copy = list(data)
            data_copy[i] = ' '
            data_copy = ''.join(data_copy)
            req = urllib2.Request(rest_url + 'atom/',data_copy)
            response = urllib2.urlopen(req).read()
            try:
                result = json.loads(response)
            except ValueError:
                print '---'
                print response
            self.assertEqual(server_process.returncode, None)

        print count
        print datetime.datetime.now()

    def notestUpdate(self):
        """ Test that update methods work """
        data = { "type":"ConceptNode",
            "name":"To be updated",
            "truthvalue":
            {"simple": {"str":0.5, "count":10}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"CountTV",
            "truthvalue":
            {"count": {"str":0.5, "count":10, "conf":0.5}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"IndefiniteTV",
            "truthvalue":
            {"indefinite": {"l":0.5, "u":0.7, "conf":0.2}}
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

        data = { "type":"ConceptNode",
            "name":"CompositeTV",
            "truthvalue":
            {"composite": {"primary": { "simple": {"str":0.5, "count":10}},
                "CONTEXTUAL":[1, {"simple": {"str":0.5, "count":10}}] }
            }
        }
        req = urllib2.Request(rest_url + 'atom/',json.dumps(data))
        response = urllib2.urlopen(req).read()
        result = json.loads(response)
        self.assertTrue("result" in result)
        self.assertEqual(result["result"], "created")

if __name__ == "__main__":
    print "Starting REST interface test"
    spawn_server()
    # unittest.main is stupid and has sys.exit hardcoded. python 2.7 has an
    # option to disable it, but most people still use 2.6 or earlier.
    try:
        unittest.main()
    except SystemExit, e:
        print('caught exit')
        kill_server()
        raise e

