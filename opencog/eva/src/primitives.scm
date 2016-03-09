;
; primitives.scm
;
; Behavior tree primitives.
;
; Defines a set of primitive behaviors from which behavior trees can be
; constructed.  Mostly a collection of odds and ends that don't have a
; home somewhere else (but probably should).
;
; --------------------------------------------------------

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))
(use-modules (opencog python))

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
")

(use-modules (opencog eva-model))

; --------------------------------------------------------
; Some debug prints.
; The are define-public, because otherwise the
; `(GroundedPredicate "scm: print-msg")` won't work.

(define-public (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "~a\n" atom) (stv 1 1))

; Print message, and print the current interaction face-id
(define-public (print-msg-face node)
	(display (cog-name node))
	(display " with face id: ")
	(display (cog-name (car (cog-outgoing-set (cog-execute!
			(DefinedSchemaNode "Current interaction target"))))))
	(newline)
	(stv 1 1))

; Print message, then print elapsed time
(define-public (print-msg-time node time)
	(display (cog-name node))
	(display " Elapsed: ")
	(display (cog-name time))
	(display " seconds\n")
	(stv 1 1))

; --------------------------------------------------------

; Start interacting with a new face picked randomly from the crowd.
(DefineLink
	(DefinedPredicateNode "Start new interaction")
	(SequentialAndLink
		; First, pick a face at random...
		(TrueLink (PutLink
			(StateLink interaction-state (VariableNode "$face-id"))
			(DefinedSchemaNode "Select random face")))
		; Record a timestamp
		(TrueLink (DefinedSchemaNode "set interaction timestamp"))
		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Start new interaction")))
	))

;; ------

;; Select a face at random, and glance at it.
(DefineLink
	(DefinedPredicateNode "glance at random face")
	(SequentialAndLink
		(DefinedPredicateNode "Select random glance target")
		(TrueLink (PutLink
			(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
				(ListLink (VariableNode "$face")))
			(GetLink (StateLink glance-state (VariableNode "$face-id")))
		))))

;; Glance at one of the newly-arrived faces.
(DefineLink
	(DefinedSchema "glance at new person")
	(Put
		(Evaluation (GroundedPredicate "py:glance_at_face")
			(ListLink (Variable "$face")))
		; If more than one new arrival, pick one randomly.
		(RandomChoice (DefinedSchema "New arrivals"))
	))

;; Glance at the last known location of a face that is no longer visible
(DefineLink
	(DefinedSchemaNode "glance at lost face")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New departures")))


; ------------------------------------------------------

;; True if sleeping, else false.
(DefineLink
	(DefinedPredicate "Is sleeping?")
	(Equal (SetLink soma-sleeping)
		(Get (State soma-state (Variable "$x")))))

;; True if bored, else false
(DefineLink
	(DefinedPredicate "Is bored?")
	(Equal (SetLink soma-bored)
		(Get (State soma-state (Variable "$x")))))


;; ------------------------------------------------------------------
;; Main loop control
(define do-run-loop #t)

(define-public (behavior-tree-run)
"
 behavior-tree-run

 Run the Eva behavior tree main loop (in a new thread),
 Call (behavior-tree-halt) to exit the loop.
"
	(set! do-run-loop #t)
	(call-with-new-thread
		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop")))))

(define-public (behavior-tree-halt)
"
 behavior-tree-halt

 Tell the Eva behavior tree main loop thread to exit.
"
	(set! do-run-loop #f))

(define loop-count 0)
(define-public (continue-running-loop)  ; public only because its in a GPN
	(set! loop-count (+ loop-count 1))

	; Print loop count to the screen.
	; (if (eq? 0 (modulo loop-count 30))
	;	(format #t "Main loop: ~a\n" loop-count))

	; Pause for one-tenth of a second... 101 millisecs
	(usleep 101000)
	(if do-run-loop (stv 1 1) (stv 0 1)))

; Return true if the behavior loop should keep running.
(DefineLink
	(DefinedPredicate "Continue running loop?")
	(Evaluation
		(GroundedPredicate "scm:continue-running-loop") (ListLink)))

; Return true if ROS is still running.
(DefineLink
	(DefinedPredicate "ROS is running?")
	(Evaluation
		(GroundedPredicate "py:ros_is_running") (ListLink)))

; ----------------------------------------------------------------------
; Sigh. Perform manual garbage collection. This really should be
; automated. XXX TODO. (Can we get ECAN to do this for us?)
(define-public (run-behavior-tree-gc)
	(define (free-stuff)
		(sleep 1)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'SetLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ListLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'NumberNode)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ConceptNode)
		(free-stuff)
	)
	(call-with-new-thread free-stuff)
)
; ----------------------------------------------------------------------
