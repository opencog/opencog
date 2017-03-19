;
; ROS robot movement module.
;
; Provides interfaces for making physical movements, and
; for saying things.

(define-module (opencog movement))

(use-modules (opencog) (opencog atom-types))

(load "movement/orchestrate.scm")

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
