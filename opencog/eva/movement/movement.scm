;
; ROS robot movement module.
;
; Provides interfaces for making physical movements, and
; for saying things.

(define-module (opencog movement))

(use-modules (opencog) (opencog atom-types) (opencog python))

(load "movement/do-movement.scm")

; Try loading the python code from this directory; else got for the
; install directory.  This assumes that the the current directory
; is in the python sys.path.
;
; If roscore is not running, then the load will hang. Thus, to avoid the
; hang, we test to see if we can talk to roscore. If we cannot, then load
; only the debug interfaces.
;
;
(define-public (start-ros-movement-node)
	(python-eval "
import os.path
import sys
import rosgraph
# This hard-coded install path from CMakefile
sys.path.append('/usr/local/share/opencog/python')
try:
    # Throw an exception if roscore is not running.
    rosgraph.Master('/rostopic').getPid()
    if (os.path.isfile('atomic.py')):
        # Python3 does not support execfile any longer
        # execfile('atomic.py'))
        exec(open('atomic.py').read())
    else:
        # execfile('/usr/local/share/opencog/python/atomic.py')
        exec(open('/usr/local/share/opencog/python/atomic.py').read())
    ros_is_running()
    print 'Loaded the OpenCog ROS Movement API'
except:
    # execfile('/usr/local/share/opencog/python/atomic-dbg.py')
    exec(open('/usr/local/share/opencog/python/atomic-dbg.py').read())
    print 'Loaded the OpenCog Movement Debug API'
"))

(start-ros-movement-node)
