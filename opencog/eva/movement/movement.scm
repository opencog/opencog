;
; ROS robot movement module.
;
; Provides interfaces for making physical movements, and
; for saying things.

(define-module (opencog movement))

(use-modules (opencog) (opencog atom-types))

(load "movement/orchestrate.scm")

; Try loading the python code from this directory;
; else go for the install directory. This is kind-of hacky;
; is there a nicer way to load stuff?
;
; (python-eval
;    "import sys\nsys.path.insert(0, '/usr/local/share/opencog/python')\n")
;
; If roscore is not running, then the load will hang. Thus, to avoid the
; hang, we test to see if we can talk to roscore. If we cannot, then load
; only the debug interfaces.
;
(define-public start-ros-movement-node
	(python-eval "
import rosgraph
try:
    # Throw an exception if roscore is not running.
    rosgraph.Master('/rostopic').getPid()
    execfile('atomic.py')
    try:
        ros_is_running()
    except NameError:
        execfile('/usr/local/share/opencog/python/atomic.py')
except:
    execfile('atomic-dbg.py')
    try:
        ros_is_running()
    except NameError:
        execfile('/usr/local/share/opencog/python/atomic-dbg.py')
"))

; If the ROS node hasn't been loaded yet, then load the "debug"
; python backend. This is needed, so that various imperative
; commands don't crash. A later load of the ROS node will harmlessly
; over-write the python entry-points.
;
; (python-eval
;   "import sys\nsys.path.insert(0, '/usr/local/share/opencog/python')\n")
;
; Hard-coded install path from CMakefile
(python-eval "
# Throws an exception if do_wake_up() is not defined.
try:
    do_wake_up()
except NameError:
    execfile('/usr/local/share/opencog/python/atomic-dbg.py')
")
