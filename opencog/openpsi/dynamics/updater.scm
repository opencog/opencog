(load "../main.scm")
;(use-modules (opencog))
(use-modules (opencog atom-types))  ;needed for AtTimeLink definition?

(define prev-value-table (make-hash-table 47))

;;; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))


; --------------------------------------------------------------

(define (psi-get-value entity)
	; todo: handle no value set (iow empty set returned from cog-execute)
	; OpenPsi values are stored using StateLinks
	(gar
		(cog-execute!

				(Get
					(State
						entity
						(Variable "$n"))))))

(define (psi-get-number-value entity)
	(define numlink (psi-get-value entity))
	(if (eq? 'NumberNode (cog-type numlink))
		(string->number (cog-name numlink))))

(define (psi-set-value entity value)
	; OpenPsi values are stored using StateLinks
	(State
		entity
		value))


;;; Modulator Creation
(define (create-openpsi-modulator name initial-value)
    (define mod
        (Concept (string-append psi-prefix-str name) (stv initial-value 1)))
    (Inheritance
        mod
        (Concept (string-append psi-prefix-str "Modulator")))
    ; todo: the below should probably be set as part of the updater init
    (hash-set! prev-value-table mod initial-value)
    mod)

(define arousal (create-openpsi-modulator "arousal" .5))

;;; SEC Creation
(define (create-openpsi-sec name)
    (define sec
        (Predicate (string-append psi-prefix-str name)))
    (Inheritance
        sec
        (Concept (string-append psi-prefix-str "SEC")))
    sec)

;;;;;;; todo: need to figure out how SEC's are going to fit into this
;; todo: make util function to define
(define power (create-openpsi-sec "Power"))

(define agent-state-power
	(List
		agent-state
		power))
(psi-set-value agent-state-power (Number .3))
(hash-set! prev-value-table agent-state-power .3)

;;; EVENT PREDICATES
(Predicate "speech-giving starts" (stv 0 1))


;;; PAU PREDICATES
; actually these will be defined somewhere else in the system
(define pau-prefix-str "PAU: ")
(define (create-pau name initial-value)
	(define pau
		(Predicate (string-append pau-prefix-str name)))
	(Inheritance
		pau
		(Concept "PAU"))
	(psi-set-value pau (Number initial-value))
	(hash-set! prev-value-table pau initial-value)
	pau)

(define voice-width
	(create-pau "voice width" .2))

;;; SYSTEM DYNAMIC INTERACTION UPDATE RULES
;; todo: move to own file

;(define psi-interaction-rule-str
;	(string-append psi-prefix-str "interaction rule"))
(define psi-interaction-rule
	(ConceptNode (string-append psi-prefix-str "interaction rule")))

(define (create-psi-interaction-rule antecedent consequent strength)
	(define rule
		(Implication
			(TypedVariable
				(Variable "$time")
				(Type "TimeNode"))
		    (AtTime
		        (Variable "$time")
			     antecedent)
		    (AtTime
		        (Variable "$time")
		        (ExecutionOutputLink
		            (GroundedSchema "scm: adjust-psi-var-level")
		            (List
		                consequent
		                (NumberNode strength)
		                antecedent)))))
	(Member rule psi-interaction-rule)
	rule
)

(define (psi-change-eval entity)
    (Evaluation
        (GroundedPredicate "scm: psi-change-in?")
        (List
            entity))
)

(define power->voice (create-psi-interaction-rule
;    (Evaluation
;        (GroundedPredicate "scm: psi-change-in?")
;        (List
;            agent-state-power))
    (psi-change-eval agent-state-power)
    voice-width
	.7)
)

; Do we want to specifiy that all antecedents will be psi-change-in? predicates?
; That would simplify the above, by just being able to pass in the entity to
; create-psi-interaction-rule. But then below format wouldn't be allowed.
; But maybe we do want to allow flexibility with the antecedent predicates.
(define speech->power (create-psi-interaction-rule
	(Evaluation
		(Predicate "speech-giving-starts"))
	agent-state-power
	.7)
)




;; todo: we are not changing the y var in proportion to the change in the x var
;; at this point. We want to add that soon.

;(TimeNode (number->string (current-time)))

;; todo:
;; maybe we want some kind of mechanism to indicate a new event is beginning
;; that persists for some specified time duration and then goes away. A recent
;; event pred? Some agent controls/updates this? A context evaluation agent? Is
;; this in the ROS behavioral control loop?

;; check out TriggerLink (some kind of trigger subscription-based messaging system

; --------------------------------------------------------------
"
 Openpsi Value Updating

 Update openpsi parameter and variable values as a funciton of the current
 value, so that increases in value are larger when the current value is low
 and smaller when the current value is high. (And vica versa for decreases.)

 Currently this function assumes the value to be updated is stored in the
 tv.strength of target-param. However, this representation maybe be changing
 subject to feedback from those in the know.

 alpha is in the range of [-1, 1] and represents the degree of change and
     whether the change is positively or negatively correlated with the
     origin parameter
     0 would lead to no change

 todo: see case-lambda in scheme for function overloading
"
(define-public (adjust-psi-var-level target alpha . origin-param)
	(display "adjust-psi-var-level\n")
	(display target)
	;(let* ((value (string->number (cog-name (psi-get-value target))))
	(let* ((value (psi-get-number-value target))
		   ;(strength (cog-stv-strength target))
		   ;(confidence (cog-stv-confidence target))
		   ; todo: replace this with a function
		   (new-value (min 1 (max 0 (+ value
				(string->number (cog-name alpha))))))
		  )

		(psi-set-value target (Number new-value))
		;(cog-set-tv! target (cog-new-stv new-strength confidence))
		(display "new vale: ")(display new-value)(newline)
	)
)




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
    (usleep 1000000))


(define (psi-get-interaction-rules)
			(cog-execute!
				(Get
					(Member
						(Variable "$rule")
						psi-interaction-rule)))
)

; ----------------------------------------------------------------------
(define-public (do-psi-updater-step)
"
  The main function that defines the steps to be taken in every cycle.
"
	(set! psi-updater-loop-count (+ psi-updater-loop-count 1))
	(display "loop count: ")(display psi-updater-loop-count)(newline)

	; grab and evaluate the interaction rules
	(let ((rules psi-get-interaction-rules))
		(map psi-evaluate-interaction-rule rules)
	)



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

(define-public (psi-change-in? target)
	(display "psi-change-in? argument: ")(display target)
	(let* ((previous (hash-ref prev-value-table target))
		   (current (psi-get-number-value target)))

		(display previous)(newline)(display current)(newline)
		(if (not (equal? previous current))
			(stv 1 1)
			(stv 0 1))
	)


)


(define (psi-evaluate-interaction-rule rule)
	; Assumes rule is of the form (Implication (TypedVar...) (AtTime ) (AtTime))
	(define antecedent (gdr (gdr rule)))
	(display antecedent)

	; starting out not using time server but will add
	(if (equal? (tv-mean (cog-evaluate! antecedent)) 1.0)
		(let ((consequent (gdr (car (cdr (cdr (cog-outgoing-set rule)))))))
			(display consequent)
			(cog-execute! consequent)
		)
	)



)


(define (psi-updater-init)
	; Create list of the changed-predicates based on the rules?
	; Alternatively we can check for change for each occurence in the rules

	(display "<insert init psi-updater here>\n")
)

(define change-predicates)
(define (psi-update-change-predicates)
	(display "<insert update change-predicates here>\n")
)

(define-public (psi-evaluate-interaction-rules)
	(display "placeholder")
	; evaluate roles either by:
	;     a) iterate each rule
	;     b) based on changed predicates


	; Update prev-value-table entries for contents of changed-predicates list

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
; Shortcuts for dev use

(define halt psi-updater-halt)
(define rule power->voice)
(define voice voice-width)
(define value psi-get-value)

;for fun
(psi-set-value agent-state-power (Number .7))


; --------------------------------------------------------------
; --------------------------------------------------------------
#! Old Code

!#