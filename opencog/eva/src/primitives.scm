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
(use-modules (opencog openpsi))

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

;; ------------------------------------------------------------------
;; Personality-related timeouts.
;;
;; According to her personality parameters, she will get bored after
;; a while, fall asleep, etc. The code immediately below defines a
;; a bunch of predicates that give yes/no answers to questions such
;; as "have I been bored for too long?".  These depend on having
;; timestamps being correctly defined and recorded at various points
;; during behavior script execution.  They also depend on the config
;; files specifying a min and max timeout value.
;
; ----------------
; change-template - utility to create timeout predicates.
;
; Define a predicate that evaluates to true or false, if it is time
; to do something. PRED-NAME is the name given to the predicate,
; TS-NAME is the name given to the timestamp that holds the start-time;
; MIN-NAME and MAX-NAME are the string names of the configurable
; min and max bounds for the time interval.  So, if the elapsed time
; since the timestamp is less than MIN, then return false; if the
; elapsed time is greater than MAX then return true; else return a
; with increasing random liklihood.
;
; To compute the likelihood correctly, we have to compute the
; integral since the last time that this check was made. Thus,
; if we are calling this predicate 10 times a second, the probability
; of a transition will be failry small; but if we call it once a second,
; the probability will be ten times greater... computing even this
; simple integral in atomese is painful. Yuck. But we have to do it.
(define (change-template pred-name ts-name min-name max-name)
	(define get-ts (string-append "get " ts-name " timestamp"))
	(define prev-ts (string-append "previous-" ts-name "-call"))
	(define delta-ts (string-append "delta-" ts-name "-time"))
	(DefineLink
		(DefinedPredicate pred-name)
		(SequentialOr
			; If elapsed time greater than max, then true.
			(GreaterThan
				; Minus computes number of seconds since interaction start.
				(Minus (TimeLink) (DefinedSchema get-ts))
				(Get (State (Schema max-name) (Variable "$max")))
			)
			(SequentialAnd
				; Delta is the time since the last check.
				(True (Put (State (Schema delta-ts) (Variable "$x"))
						(Minus (TimeLink)
							(Get (State (Schema prev-ts) (Variable "$p"))))))
				; Update time of last check to now. Must record this
				; timestamp before the min-time rejection, below.
				(True (Put (State (Schema prev-ts) (Variable "$x")) (TimeLink)))

				; If elapsed time less than min, then false.
				(GreaterThan
					; Minus computes number of seconds since interaction start.
					(Minus (TimeLink) (DefinedSchema get-ts))
					(Get (State (Schema min-name) (Variable "$min")))
				)

				; Compute integral: how long since last check?
				; Perform a pro-rated coin flip. If it is only a very short
				; time since we were last called, it is very unlikely that
				; well the random number will come up heads.  But if its
				; been a long time, then very likely the coin will come up
				; heads.
				(GreaterThan
					(Get (State (Schema delta-ts) (Variable "$delta")))
					; Random number in the configured range.
					(RandomNumber
						(Number 0)
						(Minus
							(Get (State (Schema max-name) (Variable "$max")))
							(Get (State (Schema min-name) (Variable "$min")))))
				)
			)
	)))

; Return true if it is time to interact with someone else.
(change-template "Time to change interaction" "interaction"
	"time_to_change_face_target_min" "time_to_change_face_target_max")

; Return true if we've been sleeping for long enough (i.e. longer than
; the time_to_wake_up parameter).
(change-template "Time to wake up" "sleep"
	"time_sleeping_min" "time_sleeping_max")

; Return true if we've been bored for a long time (i.e. longer than
; the time_bored_to_sleep parameter).
(change-template "Bored too long" "bored"
	"time_boredom_min" "time_boredom_max")

; Return true, if we have not heard anything in a while.
(change-template "Silent too long" "heard-something"
	"silence_min" "silence_max")

;; Evaluate to true, if an expression should be shown.
;; (if it is OK to show a new expression). Prevents system from
;; showing new facial expressions too frequently.
(change-template "Time to change expression" "expression"
	"time_since_last_expr_min" "time_since_last_expr_max")

(change-template "Time to make gesture" "gesture"
	"time_since_last_gesture_min" "time_since_last_gesture_max")

(change-template "Time to change gaze" "attn-search"
	"time_search_attn_min" "time_search_attn_max")

;; ------------------------------------------------------------------
;; Main loop control
(define do-run-loop #t)

;(define-public (behavior-tree-run)
;"
; behavior-tree-run

; Run the Eva behavior tree main loop (in a new thread),
; Call (behavior-tree-halt) to exit the loop.
;"
;	(set! do-run-loop #t)
;	(call-with-new-thread
;		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop")))))
(define-public (behavior-tree-run)
"
 behavior-tree-run

 Run the Eva behavior tree main loop (in a new thread),
 Call (behavior-tree-halt) to exit the loop.
"
	(set! do-run-loop #t)
	(call-with-new-thread
		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop"))))
	(psi-run)
	;(while #t ((usleep 100000)(psi-step)) ) causes crash

)

(define-public (behavior-tree-halt)
"
 behavior-tree-halt

 Tell the Eva behavior tree main loop thread to exit.
"
	(set! do-run-loop #f))


(define-public (behavior-tree-running?)
"
 behavior-tree-running?

 Return #t if the behavior tree is running, else return false.
"
	do-run-loop)

(define-public (behavior-tree-loop-count)
"
 behavior-tree-loop-count

 Return the loop-count of the behavior tree.
"
	loop-count)


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
