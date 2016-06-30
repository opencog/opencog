;
; updater.scm
;
; Code responsible for updating various openpsi related entities, modulators,
; and parameters based on interaction rules governing the dynamic relationships
; between them.
;
; The interaction rules specify internal dynamic relationships of the form:
;  "change in trigger-entity results in change in target-entity with strength s"
; See interaction-rules.scm for more information
;
; The updater runs in a loop, and for each change in a trigger entity, it should
; execute each and every rule containing the trigger as an antecedent, and each
; of those rules should be executed once and only once for a given change.
;

(load "../main.scm")
(load "utilities.scm")
(load "entities-temp.scm")
(load "interaction-rules.scm")

; parameter for the impact of the change in trigger variable on target
(define psi-max-strength-multiplier 5)

; Todo: implement this table in the atomspace
(define prev-value-table (make-hash-table 40))

; --------------------------------------------------------------

; List of entities that are antecedants in the interaction rules
(define psi-monitored-entities '())

(define (psi-updater-init)
"
  Initialization of the updater. Called by psi-updater-run.
"
	; Grab all variables in the antecedents of the interaction rules that are
	; arguments to (Predicate "psi-changed") and put them in
	; psi-montored-params. Populate the prev-value-table with their current
	; values.
	(define rules (psi-get-interaction-rules))
	(define evals-with-change-pred
		(cog-filter-hypergraph psi-changed-eval? (Set rules)))

	(for-each (lambda (eval-link)
				(define key (gadr eval-link))
				(define value (psi-get-number-value key))
				(hash-set! prev-value-table key value)
				(set! psi-monitored-entities (append psi-monitored-entities
												  (list key)))
			  )
			evals-with-change-pred)

	(set! psi-monitored-entities (delete-duplicates psi-monitored-entities))

	;(for-each (get-value-and-store key) evals-with-change-pred)

	;(format #t "psi-evals-with-change-pred: ~a\n" evals-with-change-pred)

)

; ----------------------------------------------------------------------

(define-public (do-psi-updater-step)
"
  Main function that executes the actions to be taken in every cycle.
  At each step:
    1) Evaluate the monitored entities and set their 'changed' predicates, and
       for each changed entity store it's value in a current-value-table (this,
       or otherwise storing the change magnitude, is needed because the current
       value of a trigger entity might change because of application of a
       previous rule.)
    2) Store the current value or change magnitude before firing the rules
       because the current value of the trigger might change as a result of a
       previous rule.
    3) Execute the interaction rules
    4) Update the previous values of the changed monitored params
"
	(define changed-params '())
	(define current-values-table (make-hash-table 20))

	(define (set-param-change-status param)
		(define tv)
		;(define changed (psi-change-in? param))
		;(format #t "\npsi-change-in? RETURN: ~a\n" changed)
		(if (psi-change-in? param)
	            (begin
	                (format "*** Found change in : ~a\n" param)
	                (set! changed-params
	                    (append changed-params (list param)))
	                (set! tv (stv 1 1))
	                (hash-set! current-values-table param (psi-get-number-value param)))
	            (set! tv (stv 0 1)))
        (Evaluation tv
            (Predicate "psi-changed")
            (List
                param)))

	(set! psi-updater-loop-count (+ psi-updater-loop-count 1))
	(let ((output-port (open-file "psilog.txt" "a")))
			(format output-port "---------------------------------------- Loop ~a\n"
				psi-updater-loop-count)
			(format output-port
				"agent power: ~a  arousal: ~a  speech: ~a  voice: ~a\n"
				(psi-get-number-value agent-state-power)
				(psi-get-number-value arousal)
				(psi-get-number-value speech)
				(psi-get-number-value voice-width))


			(close-output-port output-port))

	; Evaluate the monitored params and set "changed" predicates accordingly
	(set! changed-params '())
	(for-each set-param-change-status psi-monitored-entities)
	;(format #t "\nchanged-params: ~a\n\n" changed-params)


	; grab and evaluate the interaction rules
	; todo: Could optimize by only calling rules containing the changed params
	(let ((rules (psi-get-interaction-rules)))
		(map psi-evaluate-interaction-rule rules)
	)

	; Update prev-value-table entries for the changed (monitored) params
	(for-each (lambda (param)
					(define value (hash-ref current-values-table param))
					(hash-set! prev-value-table param value))
			  changed-params)


	(psi-pause)
	(stv 1 1)
)




; --------------------------------------------------------------
(define-public (adjust-psi-var-level target strength trigger)
"
 Adjust psi-related variable based on triggered interaction rule

 Update openpsi parameter and variable values as a functon of the current
 value of the target and the strength of the interaction rule. The current value
 of the target influences the change magnitude such that increases in value are
 larger when the current value is low and smaller when the current value is
 high. (And vica versa for decreases.)

 Currently the implementation assumes the openpsi parameters are
 normalized in [0 1]. The values of the params are assumed to be stored
 in a StateLink or if not in a StateLink an attempt is made to evaluate or
 execute the atom to obtain a value (see psi-get-value function). However, this
 representation maybe be changing subject to feedback from those in the know.

  target - the entity to update
  strength - the strength of the interaction rule
  trigger - the entity that changed that triggered the interaction rule
"
	(define new-value)
	(define strength-multiplier)
	(define alpha)
	(define slope)
	(display "\n------------------------------------------------------\n")
	(format #t "adjust-psi-var-level   target: ~a   trigger: ~a"
		target trigger)
	(let* ((current-value (psi-get-number-value target))
		   (trigger-change (psi-get-change-magnitude trigger))
		  )

		; Determine normalized value for the alpha for the change function.
		; Alpha is based on the strength of the interaction and the
		; magnititude of change in the trigger.

		; Assume interaction strength .5 means a roughly one-to-one
		; change in magnititude between the trigger and target. IOW a given
		; change in trigger results in a change of approximately equal
		; magnitude in the target.
		; WAIT - maybe just have interaction strength be the multiplier ?
		; We could assume strength = 1 by default if not present and make
		; it optional, but not sure how that would/wouldn't work with the
		; rules.

		(set! strength (string->number (cog-name strength)))
		; Strength needs to be in [-1, 1]
		(set! strength (max (min strength 1) -1))

		; We want to map the strength to some multiplier, where .5 strength
		; maps to multiplier of 1, and strength 1 maps to some max multiplier
		; Probably want some exponential function that passes through (0,0),
		; (.5,1), and (1,MAX), but in the meantime:
		(if (<= strength .5)
			(set! strength-multiplier (* 2 strength))

			; else strength is > .5 so set multiplier between 1 and some specified max
			; using the formula that god only knows how i came up with but i
			; think actually works.
			(let ((max psi-max-strength-multiplier))
				(set! strength-multiplier
					(- (+ (* (- (* 2 max) 2) strength) 2) max)))
		)

		(set! alpha (* trigger-change strength-multiplier))
		; make sure alpha is in [-1,1]
		(if (> alpha 0)
			(set! alpha (min alpha 1))
			(set! alpha (max alpha -1)))

		(format #t "strength: ~a     change: ~a       alpha: ~a\n"
			strength trigger-change alpha)

		; Increasing slope increases the degree change at all levels
		(set! slope 10000)
		; finagle the extreme cases because the curve is not exactly how we want
		(if (> alpha 0)
			(if (>= alpha .5)
				(set! current-value (max current-value .2))
				(set! current-value (max current-value .1))))
		(if (< alpha 0)
			(if (<= alpha -.5)
				(set! current-value (min current-value .8))
				(set! current-value (min current-value .9))))

		; The formula is: (slope^(a*x) - 1) / (slope^a -1)
		; For this formula alpha needs to be negative for increases
		; and positive for decreases. And it can't be 0.
		(set! alpha (* alpha -1))

		; alpha == 0 means no change
		(if (= alpha 0)
			(set! new-value current-value)
			(set! new-value (/ (- (expt slope (* alpha current-value)) 1)
				(- (expt slope alpha) 1))))

		; An older version formula using tanh
		;(set! new-value (max .15 (tanh (* 3.141592654 current-value))))
		; tanh never gets to one so set it to one above a certain point
		;(if (> new-value .95)
		;	(set! new-value 1))
		(psi-set-value! target (Number new-value))
		;(cog-set-tv! target (cog-new-stv new-strength confidence))
		(format #t "current value: ~a    new value: ~a\n" current-value new-value)

		; Todo: create one-way direction rules (e.g., only for a trigger increase,)
		; but not for a decrease. Could implement with "increase-in" and
		; decrease-in predicates which are evaluated and set at the same time
		; as the change-in predicates.

	)
)

; --------------------------------------------------------------
; Helper functions

(define (psi-get-interaction-rules)
	(cog-outgoing-set
			(cog-execute!
				(Get
					(Member
						(Variable "$rule")
						psi-interaction-rule)))))

; helper to identify (Eval (GPN "scm: psi-change-in?" (List . . .))) links
; not used anymore i think
(define (psi-change-eval? atom)
	(if (and (equal? (cog-type atom) 'EvaluationLink)
			 (equal? (gar atom) (GroundedPredicateNode "scm: psi-change-in?")))
		#t
		#f))

; helper to identify (Eval (Predicate "psi-changed" (List . . .))) links
(define (psi-changed-eval? atom)
	(if (and (equal? (cog-type atom) 'EvaluationLink)
			 (equal? (gar atom) (Predicate "psi-changed")))
		#t
		#f))

(define (psi-change-in? target)
	;(display "psi-change-in? argument: \n")(display target)
	(let* ((previous (hash-ref prev-value-table target))
		   (current (psi-get-number-value target)))

		;(format #t "previous value: ~a   current value: ~a\n" previous current)
		(if (and
				; current value is defined
				(not (equal? #f current))
				(not (equal? previous current)))
			#t
			#f
			; from when function was called as a GPN
			;(stv 1 1)
			;(stv 0 1)
		)
	)
)

(define (psi-get-change-magnitude target)
  "
  Returns the normalized magnitude change in target from its previous value.
  Assumes (for now) that the values are normalized in [0,1]
  "
    (define return)
    ;(format #t "psi-get-change-magnitude target: ~a" target)
	(let* ((previous (hash-ref prev-value-table target))
		   (current (psi-get-number-value target)))
		(if (and (number? previous) (number? current))
			(set! return (- current previous))
			(set! return 0))
		;(display previous)
		;(display current)
		;(display return)
		;(format #t "previous: ~a   current: ~a   return: ~a\n"
		;	previous current return)
		return)
)

(define (psi-evaluate-interaction-rule rule)
	; Assumes rule is of the form (PredictiveImplication AtTime ...)
	(define antecedent (gdr rule))
	;(format #t "\nevaluating: ~a\n" antecedent)

	; starting out not using time server but will add
	(if (equal? (tv-mean (cog-evaluate! antecedent)) 1.0)
		(let ((consequent (list-ref (cog-outgoing-set rule) 2)))
			;(format #t "\n**********************************\nexecuting: ~a"
			;	consequent)
			;(format #t "~a**********************************\n" consequent)
			(cog-execute! consequent)
		)
	)
)


;temp
(define (listlink? atom)
	(if (equal? (cog-type atom) 'ListLink)
		#t
		#f))

(define (cog-filter-hypergraph pred? atom)
"
  Recursively traverse hyptergraph and return list of all atoms that satisfy
  function pred?
  (This is similar to the opencog utilities.scm filter-hypergraph function, but
  filters out links also. And also takes atom rather than scheme list as the
  parameter.)
  Todo: Add this to scheme utilities in Atomspace repo?
"
	(define results '())
	(if (pred? atom)
		(set! results (append results (list atom))))
	(if (cog-link? atom)
		(for-each (lambda (sub-atom)
					(set! results
						(append (cog-filter-hypergraph pred? sub-atom) results))
				  )
				  (cog-outgoing-set atom)))
	results
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

(define updater-continue-pred
	(Evaluation (stv 1 1)
	    (Predicate "continue-psi-updater-loop")
	    (ListLink)))

(define (psi-pause)
    ; Pause for 100 millisecs, to keep the number of loops within a reasonable
    ; range.
    ; todo: is this needed?
    ; keeping it slow for now dev purposes
    (usleep 1500000))


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

	(psi-updater-init)
    (set! psi-updater-is-running #t)
    (cog-set-tv! updater-continue-pred (stv 1 1))
    (call-with-new-thread
        (lambda () (cog-evaluate! (loop-node))))
)

(define-public (psi-updater-halt)
"
  Tells the psi loop thread, that is started by running `(psi-update-run)`, to exit.
"
    (set! psi-updater-is-running #f)
    (cog-set-tv! updater-continue-pred (stv 0 1))
)



; --------------------------------------------------------------
; Shortcuts for dev use

(define halt psi-updater-halt)
(define h halt)
(define r psi-updater-run)
(define r1 speech->power)
(define r2 power->voice)
(define voice voice-width)
(define value psi-get-number-value)
(define rules (psi-get-interaction-rules))

(define (psi-decrease-value target)
	(psi-set-value! target (Number (- (psi-get-number-value target) .1))))
(define (psi-increase-value target)
	(psi-set-value! target (Number (+ (psi-get-number-value target) .1))))

(define d psi-decrease-value)
(define i psi-increase-value)

(define (psi-set-pred-true target)
	(Evaluation target (List) (stv 1 1)))
(define (psi-set-pred-false target)
	(Evaluation target (List) (stv 0 1)))

(define t psi-set-pred-true)
(define f psi-set-pred-false)

(define s speech)
(define p agent-state-power)


(define (uupdate a x) (/ (- (expt a x) 1) (- a 1)))

; (10000^(a*x) - 1) / (10000^a -1)
(define (update2 a x) (/ (- (expt 1000 (* a x)) 1) (- (expt 1000 a) 1)))


; --------------------------------------------------------------


