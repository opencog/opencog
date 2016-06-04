(load "../main.scm")
;(use-modules (opencog))
(use-modules (opencog atom-types))  ;needed for AtTimeLink definition?

;;; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))

;;; Modulator Creation
(define (create-openpsi-modulator name initial-value)
    (define mod
        (Concept (string-append psi-prefix-str name) (stv initial-value 1)))
    (Inheritance
        param
        (Concept (string-append psi-prefix-str "Modulator")))
    mod)


;;; SEC Creation
(define (create-openpsi-sec name initial-value)
    (define sec
        (Concept (string-append psi-prefix-str name) (stv initial-value 1)))
    (Inheritance
        sec
        (Concept (string-append psi-prefix-str "SEC")))
    sec)

;;;;;;; todo: need to figure out how SEC's are going to fit into this
;; todo: make util function to define
(define power (create-openpsi-sec "Power" .5))
(define agent-state-power
	(Evaluation (stv .5 1)
		power
		agent-state))


;;; EVENT PREDICATES
(Predicate "speech-giving starts" (stv 0 1))


;;; PAU PREDICATES
; actually these will be defined somewhere else in the system
(define pau-prefix-str "PAU: ")
(define (create-pau-predicate name initial-value)
	(define pau
		(Predicate (string-append pau-prefix-str name) (stv initial-value 1)))
	(Inheritance
		pau
		(Concept "PAU"))
	pau)

(define voice-width
	(create-pau-predicate "voice width" .5))




;;; SYSTEM DYNAMIC INTERACTION UPDATE RULES
;; todo: move to own file

;; When starting a speech -> boost power
;; todo: how to represent "Starts in Interval"? ask Nil

(Implication
	(TypedVariable
		(Variable "$time")
		(Type "TimeNode"))
    (AtTime
        (Variable "$time")
        (Evaluation
	        (Predicate "speech-giving-starts")))
    (AtTime
        (Variable "$time")
        (ExecutionLink
            (DefinedSchema "adjust-openpsi-var-level")
            (List
                agent-state-power
                (NumberNode .7)))))



; When x changes --> change y
; When power changes -> adjust voice width

(Implication
	(TypedVariable
		(Variable "$time")
		(Type "TimeNode"))
    (AtTime
        (Variable "$time")
        (Evaluation
	        (Predicate "change")
	        (List
	            agent-state-power)))
    (AtTime
        (Variable "$time")
        (ExecutionLink
            (DefinedSchema "adjust-openpsi-var-level")
            (List
                voice-width
                (NumberNode .7)
                agent-state-power))))

;; todo: we are not changing the y var in proportion to the change in the x var
;; at this point. We probably want to add that soon.

;(TimeNode (number->string (current-time)))

;; todo:
;; maybe we want some kind of mechanism to indicate a new event is beginning
;; that persists for some specified time duration and then goes away. A recent
;; event pred? Some agent controls/updates this? A context evaluation agent? Is
;; this in the ROS behavioral control loop?

;; check out TriggerLink (some kind of trigger subscription-based messaging system

;; OPENPSI VALUE UPDATING
;;
;; Update openpsi parameter and variable values as a funciton of the current
;; value, so that increases in value are larger when the current value is low
;; and smaller when the current value is high. (And vica versa for decreases.)
;;
;; Currently this function assumes the value to be updated is stored in the
;; tv.strength of target-param. However, this representation maybe be changing
;; subject to feedback from those in the know.
;;
;; alpha is in the range of [-1, 1] and represents the degree of change and
;;     whether the change is positively or negatively correlated with the
;;     origin parameter
;;     0 would lead to no change
;;
;; todo: see case-lambda in scheme for function overloading
(define (adjust-openpsi-var-level target-parm alpha . origin-param)
	(let* ((strength (cog-stv-strength target-parm))
		   (confidence (cog-stv-confidence target-parm))

			; todo: replace this with a function
			(new-strength (min 1 (max 0 (+ strength
				(string->number (cog-name alpha)))))))

		(cog-set-tv! target-parm (cog-new-stv new-strength confidence))
		(display "new strength: ")(display new-strength)(newline)))



; --------------------------------------------------------------
; Updater Loop Control
; --------------------------------------------------------------
(define psi-updater-is-running #f)
(define psi-updater-loop-count 0)
(define continue-psi-updater-loop (stv 1 1))

(define-public (psi-running?)
"
  Return #t if the openpsi loop is running, else return #f.
"
    psi-updater-is-running
)

(define continue-pred
	(Evaluation (stv 1 1)
	    (Predicate "continue-psi-updater-loop")
	    (ListLink)))

(define (psi-pause)
    ; Pause for 100 millisecs, to keep the number of loops within a reasonable
    ; range.
    ; todo: is this needed?
    (usleep 100000))


; ----------------------------------------------------------------------
(define-public (do-psi-updater-step)
"
  The main function that defines the steps to be taken in every cycle.
"
	(set! psi-updater-loop-count (+ psi-updater-loop-count 1))
	(display "loop count: ")(display psi-updater-loop-count)(newline)

;    (let* ((rules (psi-select-rules)))
;        (map (lambda (x)
;                (let* ((action (psi-get-action x))
;                       (goals (psi-related-goals action)))
;                    (cog-execute! action)
;                    (map cog-evaluate! goals)))
;            rules)
;        (stv 1 1)
;    )

	(psi-pause)
	(stv 1 1)
)

; --------------------------------------------------------------
(define-public (psi-updater-run)
"
  Run `psi-step` in a new thread. Call (psi-updater-halt) to exit the loop.
"
    (define loop-name (string-append psi-prefix-str "updater-loop"))
    (define (loop-node) (DefinedPredicateNode loop-name))
    (define (define-psi-loop)
        (DefineLink
            (loop-node)
            (SatisfactionLink
                (SequentialAnd
                    (Evaluation
                        (Predicate "continue-psi-updater-loop")
                        (ListLink))
                    (Evaluation
                        (GroundedPredicate "scm: do-psi-updater-step")
                        (ListLink))
                    (loop-node)))))

    ;(if (null? (cog-node 'DefinedPredicateNode loop-name))
        (define-psi-loop)
    ;    #f ; Nothing to do already defined
    ;)

    (set! psi-updater-is-running #t)
    (call-with-new-thread
        (lambda () (cog-evaluate! (loop-node))))
)

; --------------------------------------------------------------
(define-public (psi-updater-halt)
"
  Tells the psi loop thread, that is started by running `(psi-run)`, to exit.
"
    (set! psi-updater-is-running #f)
    (cog-set-tv! continue-pred (stv 0 1))
)


; --------------------------------------------------------------
; Shortcuts

(define halt psi-updater-halt)