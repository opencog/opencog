;
; Self-awareness scheme module for the Eva robot.
; Allows the self-awareness code to be imported as a module.

(define-module (opencog eva-model))

(use-modules (opencog) (opencog atom-types)
	(opencog query) (opencog exec) (opencog python))


; Load various parts....
(load "eva-model/faces.scm")
(load "eva-model/self-model.scm")
(load "eva-model/orchestrate.scm")

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
