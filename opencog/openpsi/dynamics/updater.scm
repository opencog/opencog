(load "../main.scm")
;(use-modules (opencog))
(use-modules (opencog atom-types))  ;needed for AtTimeLink definition?

(define prev-value-table (make-hash-table 47))

;;; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))


; --------------------------------------------------------------

(define (psi-get-value entity)
	(define result #f)

	; First check for StateLink value
	; OpenPsi values are stored using StateLinks
	(define resultset
		(cog-execute!
				(Get
					(State
						entity
						(Variable "$n")))))
	(if (not (null? (cog-outgoing-set resultset)))
		(set! result (gar resultset))

		; else check if entity is an evaluation or predicate or schema
		(let ((type (cog-type entity)))
			(if (or (equal? type 'PredicateNode)
				   (equal? type 'GroundedPredicateNode)
				   (equal? type 'DefinedPredicateNode))
				(set! result (cog-evaluate! (Evaluation entity (List)))))
			(if (or (equal? type 'SchemaNode)
            		(equal? type 'GroundedSchemaNode)
            	    (equal? type 'DefinedSchemaeNode))
            	(set! result (cog-execute! (ExecutionOutput entity (List)))))
            (if (equal? type 'EvaluationLink)
                (set! result (cog-evaluate! entity)))
        )
	)
	result
)

(define (psi-get-number-value entity)
	(define result (psi-get-value entity))
	(if (and (cog-atom? result) (eq? 'NumberNode (cog-type result)))
		(set! result (string->number (cog-name result))))
	(if (cog-tv? result)
		(set! result (tv-mean result)))
	result)

(define (psi-set-value! entity value)
	; OpenPsi values are stored using StateLinks
	(State
		entity
		value))

; --------------------------------------------------------------
; Entity Creation

;;; Modulator Creation
(define (create-openpsi-modulator name initial-value)
    (define mod
        (Concept (string-append psi-prefix-str name)))
    (Inheritance
        mod
        (Concept (string-append psi-prefix-str "Modulator")))
    (psi-set-value! mod (Number initial-value))
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
(psi-set-value! agent-state-power (Number .3))
;(hash-set! prev-value-table agent-state-power .3)

;;; EVENT PREDICATES
(define speech (Predicate "speech-giving-starts"))
(Evaluation speech (List) (stv 0 1))
;(hash-set! prev-value-table speech 0.0)


;;; PAU PREDICATES
; actually these will be defined somewhere else in the system
(define pau-prefix-str "PAU: ")
(define (create-pau name initial-value)
	(define pau
		(Predicate (string-append pau-prefix-str name)))
	(Inheritance
		pau
		(Concept "PAU"))
	(psi-set-value! pau (Number initial-value))
	;(hash-set! prev-value-table pau initial-value)
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
		(PredictiveImplication
			(TimeNode 1)
		    (Evaluation
                ;(GroundedPredicate "scm: psi-change-in?")
                (Predicate "psi-changed")
                (List
                    antecedent))
		    (ExecutionOutputLink
	            (GroundedSchema "scm: adjust-psi-var-level")
	            (List
	                consequent
	                (NumberNode strength)
	                antecedent))))
	(Member rule psi-interaction-rule)
	rule
)

(define speech->power
	(create-psi-interaction-rule speech agent-state-power .3)
)

(define power->voice
	(create-psi-interaction-rule agent-state-power voice-width .1))

(define power->arousal
	(create-psi-interaction-rule agent-state-power arousal .2))

(define arousal->voice
	(create-psi-interaction-rule arousal voice-width .1))



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

 Update openpsi parameter and variable values as a functon of the current
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
(define-public (adjust-psi-var-level target strength . origin)
	(define new-value)
	(set! origin (list-ref origin 0))
	(format #t "adjust-psi-var-level   strength: ~a   target: ~a   origin: ~a"
		strength target origin)
	(let* ((value (psi-get-number-value target))
		   ; todo: replace this with a function
		   ; Determine normalized value for the alpha for the change function.
		   ; Alpha is based on the strength of the interaction and the
		   ; magnititude of change in the origin.

		   ; Assume interaction strength .5 means a roughly one-to-one
		   ; change in magnititude between the origin and target. IOW a given
		   ; change in origin results in a change of approximately equal
		   ; magnitude in the target.
		   ; WAIT - why not just have alpha be the multiplier duh
		   ; We could even assume alpha = 1 by default if not present and make
		   ; it optional, but not sure how that would/wouldn't work with the
		   ; rules.

		   (origin-change (psi-get-change-magnitude origin))
		   (alpha (* (string->number (cog-name strength)) origin-change))
		  )
		(set! new-value (max .15 (tanh (* 3.141592654 value))))
		; tanh never gets to one so set it to one above a certain point
		(if (> new-value .99)
			(set! new-value 1))
		(psi-set-value! target (Number new-value))
		;(cog-set-tv! target (cog-new-stv new-strength confidence))
		(format #t "previous value: ~a    new value: ~a\n" value new-value)
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

(define psi-monitored-params '())

(define (psi-updater-init)
	; Grab all variables in the antecedents of the interaction rules that are
	; arguments to (Predicate "psi-changed") and put them in
	; psi-monintered--params. Populate the prev-value-table with their current
	; values.

	; Populate prev-values-table based on current value of the arguments to
	; (GroupdedPredicate "psi-change-in") in the interaction rules
	; Grab the interaction rules and get the eval links
	(define rules (psi-get-interaction-rules))
	(define evals-with-change-pred (cog-filter-hypergraph psi-changed-eval? (Set rules)))
	;(define change-evals (cog-filter-hypergraph psi-change-eval? (Set rules)))

	(for-each (lambda (eval-link)
				(define key (gadr eval-link))
				(define value (psi-get-value key))
				(hash-set! prev-value-table key value)
				(set! psi-monitored-params (append psi-monitored-params
												  (list key)))
			  )
			evals-with-change-pred)

	(set! psi-monitored-params (delete-duplicates psi-monitored-params))

	;(for-each (get-value-and-store key) evals-with-change-pred)

	;(format #t "psi-evals-with-change-pred: ~a\n" evals-with-change-pred)

)

; ----------------------------------------------------------------------

(define-public (do-psi-updater-step)
"
  The main function that defines the steps to be taken in every cycle.
  At each step:
    1) Evaluate the monitored params and set their 'changed' predicates
    2) Fire up the interaction rules
    3) Update the previous values of the changed monitored params
"
	(define changed-params '())
	(define current-values-table (make-hash-table 20))

	(define (set-param-change-status param)
		(define tv)
		;(define changed (psi-change-in? param))
		;(format #t "\npsi-change-in? RETURN: ~a\n" changed)
		(if (psi-change-in? param)
	            (begin
	                (set! changed-params
	                    (append changed-params (list param)))
	                (set! tv (stv 1 1))
	                (hash-set! current-values-table param (psi-get-value param)))
	            (set! tv (stv 0 1)))
        (Evaluation tv
            (Predicate "psi-changed")
            (List
                param)))

	(set! psi-updater-loop-count (+ psi-updater-loop-count 1))
	(let ((output-port (open-file "psilog.txt" "a")))
			(format output-port "\n---------------------------------------- Loop ~a\n"
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
	(for-each set-param-change-status psi-monitored-params)
	;(format #t "\nchanged-params: ~a\n\n" changed-params)


	; grab and evaluate the interaction rules
	; todo: Could optimize by only calling rules containing the changed params
	; todo: Ideally adjust-psi-var-level should use the current-vales-table
	;   rather than grabbing the value dynamically from the atomspace, because
	;   the current value might have changed from previous rules.
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

(define (psi-change-in? target)
	;(display "psi-change-in? argument: \n")(display target)
	(let* ((previous (hash-ref prev-value-table target))
		   (current (psi-get-value target)))

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
  Todo: Assuming for now that the values are normalized in [0,1]
  "
	(let* ((previous (hash-ref prev-value-table target))
		   (current (psi-get-number-value target)))
		(if (and (number? previous) (number? current))
			(- previous current)
			0)))


(define (psi-evaluate-interaction-rule rule)
	; Assumes rule is of the form (PredictiveImplication AtTime ...)
	(define antecedent (gdr rule))
	;(format #t "\nevaluating: ~a\n" antecedent)

	; starting out not using time server but will add
	(if (equal? (tv-mean (cog-evaluate! antecedent)) 1.0)
		(let ((consequent (list-ref (cog-outgoing-set rule) 2)))
			(format #t "\n**********************************\nexecuting: ~a"
				consequent)
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
"
	(define results '())
	(if (pred? atom)
		(set! results (append results (list atom))))
	(if (cog-link? atom)
		(for-each (lambda (sub-atom)
					(set! results (append (cog-filter-hypergraph pred? sub-atom) results))
				  )
				  (cog-outgoing-set atom)))
	results
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
(define value psi-get-value)
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


