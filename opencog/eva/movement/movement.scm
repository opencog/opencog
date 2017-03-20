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
(define-public start-ros-movement-node
	(python-eval "
import os.path
import sys
import rosgraph
sys.path.append('/usr/local/share/opencog/python')
try:
    # Throw an exception if roscore is not running.
    rosgraph.Master('/rostopic').getPid()
    if (os.path.isfile('atomic.py')):
        execfile('atomic.py')
    else:
        execfile('/usr/local/share/opencog/python/atomic.py')
    ros_is_running()
    print 'Loaded the OpenCog ROS Movement API'
except:
    execfile('/usr/local/share/opencog/python/atomic-dbg.py')
    print 'Loaded the OpenCog Movement Debug API'
"))

; If the ROS node hasn't been loaded yet, then load the "debug"
; python backend. This is needed, so that various imperative
; commands don't crash. A later load of the ROS node will harmlessly
; over-write the python entry-points.
;
; Hard-coded install path from CMakefile
(python-eval "
# Throws an exception if do_wake_up() is not defined.
try:
    do_wake_up()
except NameError:
    execfile('/usr/local/share/opencog/python/atomic-dbg.py')
")

; ... well, above should have loaded the debug interfaces.
; Now check for ROS, and if it is up and running, then load
; the ROS interfaces.
(start-ros-movement-node)
