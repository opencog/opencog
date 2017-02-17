;
; updater.scm
;
; Copyright 2106 OpenCog Foundation`
;
; Code responsible for updating various openpsi related entities,
; modulators, and parameters based on interaction rules governing the
; dynamic relationships between them.
;
; The interaction rules specify internal dynamic relationships of the form:
; "change in trigger-entity causes change in target-entity with strength s"
; See interaction-rule.scm for more information.
;
; The updater runs in a loop, and, for each change in a trigger entity,
; it will execute every rule containing that trigger as an antecedent.
; Each rule will be executed once, for a given change.
;
; To use:
; (use-modules (opencog openpsi))
; (psi-updater-run)  ; to start the loop
; (psi-updater-halt) ; to stop the loop
;
; If (define logging #t) is set below, then
; > tail -f "psilog.txt"
; can be used to monitor changes in some select events and variables. (It's
; pretty crude.)
;
; Instructions on using this module, including creation of rules and event
; event detection, are in README.md under the section "Modulators and Internal
; Dynamics."

(load "utilities.scm")
(load "modulator.scm")
(load "sec.scm")
(load "interaction-rule.scm")
(load "event.scm")

(define logging #t)
(define verbose #f)
(define slo-mo #f)
(define single-step #f)

; --------------------------------------------------------------
; Config Parameters
; Todo: Move these to a config file

; Multiplier that can be tweaked to increase or decrease the sensitivity of
; interactactions between openpsi entities. Default is 1.
(define dynamics-sensitivity 1)

; Parameter to tweak the strength of impact of changes in triggers on changes in
; targets.
(define psi-max-strength-multiplier 5)

; Number of steps after a new event has occurred to fire off the expression
; callback. This is needed to allow the effects of interaction rules to
; propagate through the system after an event has occurred.
(define psi-expression-loop-delay 4)

; Rate of decay of modulators and secs toward their baseline values at each time 
; step.
(define decay-rate .001)
(define decay-factor (- 1 decay-rate))

; --------------------------------------------------------------

; Todo: implement these tables in the atomspace
(define prev-value-table (make-hash-table 40))
(define prev-most-recent-ts-table (make-hash-table 40))
(define psi-event-detection-callbacks '())

; --------------------------------------------------------------
;
; Call back function for client to convert psi params to physiogical and
; emotional expression.
(define psi-expression-callback)

(define psi-event-detected-pred (Predicate "psi-event-detected"))
(define psi-event-at-loop-num-node (Concept "psi-event-at-loop-num"))

; List of entities that are antecedants in the interaction rules
(define psi-monitored-entities '())
(define psi-monitored-events '())

; List of all modulators and sec's
(define psi-modulators-and-secs '())

(define (psi-set-expression-callback! callback)
"
  psi-set-expression-callback! CALLBACK - set the callback that
  triggers emotion expression update. (? Huh? please explain.)

  CALLBACK is supplied by the client and gets called when OpenPsi
  believes a change of emotional expression based on psi values
  would be good to do (e.g. after a new event is detected and psi
  params have been updated).
"
	(set! psi-expression-callback callback)
)

(define-public (psi-set-event-callback! callback)
"
  psi-set-event-callback! CALLBACK - Set function for event detection.
"
	; Save callback function for event detection
	(set! psi-event-detection-callbacks
		(append psi-event-detection-callbacks (list callback)))
)

; Todo: Add a general callback function that is called once each loop

(define (psi-updater-init)
"
  Initialization of the updater. Called by psi-updater-run.

  Initializes list of monitered entities and events, and table of
  their previous values.
"
	; Grab all variables in the antecedents of the interaction rules
	; that are arguments to (Predicate "psi-changed") and put them in
	; psi-montored-params. Populate the prev-value-table with their
	; current values.
	(define rules (psi-get-interaction-rules))
	(define evals-with-change-pred
		(cog-filter-hypergraph psi-changed-eval? (Set rules)))

	(if verbose (display "psi-updater-init\n"))

	; Init previous value table for the monitored entities
	(for-each 
		(lambda (eval-link)
			(define key (gadr eval-link))
			(define value (psi-get-number-value key))
			(hash-set! prev-value-table key value)
			(set! psi-monitored-entities (append psi-monitored-entities 
				(list key)))
		)
		evals-with-change-pred)

	; If we are logging for debugging, then add all the modulators and
	; sec's to the list of entities that get monitored so they will be
	; flagged as changed in the output.
	(if logging
		(set! psi-monitored-entities
			(delete-duplicates
				(append!
					(psi-get-modulators) (psi-get-secs) psi-monitored-entities)))
	)

	(set! psi-monitored-entities (delete-duplicates psi-monitored-entities))
	(if verbose (format #t "monitored entities: ~a\n" psi-monitored-entities))

	(set! psi-modulators-and-secs (append! (psi-get-modulators) (psi-get-secs)))

	; Set most-recent timestamps for events
	(set! psi-monitored-events (psi-get-monitored-events))
	(if verbose (format #t "monitored events: ~a\n" psi-monitored-events))
	(for-each
		(lambda (event)
			(hash-set! prev-most-recent-ts-table
				 event (psi-get-most-recent-ts event))
			;(set! psi-monitored-events (append psi-monitored-events
			;								(list event)))
		)
		psi-monitored-events)

	; Set event detected loop count to 0
	(psi-set-value! psi-event-at-loop-num-node 0)

	(if logging
		(let ((output-port (open-file "psilog.txt" "a")))
			(format output-port "\n\n\n--- New Session ---\n\n")))

	;(format #t "psi-evals-with-change-pred: ~a\n" evals-with-change-pred)
)

; ----------------------------------------------------------------------

(define-public (do-psi-updater-step)
"
  Main function that executes the actions to be taken in every cycle.
  At each step:
   (1) Evaluate the monitored events and set their 'event-detected' predicates
   (2) Evaluate the monitored entities and set their 'changed' predicates, and
       for each changed entity store it's value in a value-at-step-start (this,
       or otherwise storing the change magnitude, is needed because the current
       value of a trigger entity might change because of application of a
       previous rule.)
   (3) Store the current value or change magnitude before firing the rules
       because the current value of the trigger might change as a result of a
       previous rule.
   (4) Execute the interaction rules
   (5) Decay variables toward their baseline value
   (6) Update the previous values of the changed monitored params
   (7) Set the previous values of the event predicates to 0, because each
       particular instance of an event should only fire rules at a single step.
"
	(define changed-params '())
	(define detected-events '())
	(define value-at-step-start (make-hash-table 20))

	; These are used just for display highlighting for debug output
	(define monitored-pau '())
	(define changed-pau '())

	(define (set-param-change-status param)
		(define changed-tv)
		(define pos-change-tv)
		(define neg-change-tv)
		(define current-val)
		(define previous-val)
		;(format #t "set-param-change-status: ~a  psi-change-in?: ~a\n"
		;	param (psi-change-in? param))
		(if (psi-change-in? param)
			(begin
				(if verbose
					(format #t "\n*** Found change in : ~a   loop ~a\n" param
						psi-updater-loop-count))
				(set! changed-params
					(append changed-params (list param)))
				(set! previous-val (hash-ref prev-value-table param))
				(set! current-val (psi-get-number-value param))
				(if verbose
					(format #t "previous: ~a  current: ~a\n" previous-val
						current-val))
				; If previous-val was #f (iow not set), assume previous value
				; to be 0? Doing that for now, but not sure if this is best.
				(if (eq? previous-val #f)
					(set! previous-val 0))
				(hash-set! value-at-step-start param
					(psi-get-number-value param))
				(set! changed-tv (stv 1 1))
				(if (> current-val previous-val)
					(begin
						(set! pos-change-tv (stv 1 1))
						(set! neg-change-tv (stv 0 1)))
					(begin
						(set! pos-change-tv (stv 0 1))
						(set! neg-change-tv (stv 1 1)))))

			(begin ; no change in the param
				(set! changed-tv (stv 0 1))
				(set! pos-change-tv (stv 0 1))
				(set! neg-change-tv (stv 0 1))))

		(Evaluation changed-tv
			(Predicate "psi-changed")
			(List
				param))

		(Evaluation pos-change-tv
			(Predicate "psi-increased")
			(List
				param))

		(Evaluation neg-change-tv
			(Predicate "psi-decreased")
			(List
				param))
	)

	(define (set-new-event-status event)
		(define prev-most-recent-ts (hash-ref prev-most-recent-ts-table event))
		(define curr-most-recent-ts (psi-get-most-recent-ts event))
		(define new-event-tv)
		;(format #t "set-new-event-status event: ~a" event)
		;(format #t "prev-most-recent-ts: ~a   curr-most-recent-ts: ~a\n"
		;	prev-most-recent-ts curr-most-recent-ts)
		(if (not (equal? prev-most-recent-ts curr-most-recent-ts))
			(begin
				(if verbose
					(format #t "\n*** Detected event occurrence: ~a  loop ~a\n"
						event psi-updater-loop-count)
				)
				(set! detected-events
					(append detected-events (list event)))

				; set the event value to one
				(psi-set-value! event 1)
				; replace previous-latest-ts with current-latest-ts
				(hash-set! prev-most-recent-ts-table event curr-most-recent-ts)
			)
		)
	)

	(define (highlight-display var)
		; add astericks to highlight changed values
		(define value (psi-get-number-value var))
		(if (number? value)
			(if (or (member var changed-params)
					(member var changed-pau))
				(format #f "*~1,2f*" value)
				(format #f "~1,2f" value))
			; value is not a number, just return it
			value))


	(set! psi-updater-loop-count (+ psi-updater-loop-count 1))


	; Event Detection
	; First call the event detection callback functions
	;(format #t "psi-event-detection-callbacks: ~a\n"
	;	psi-event-detection-callbacks)
	(for-each (lambda (f) (apply f '())) psi-event-detection-callbacks)

	; Evaluate the monitored events and set "new-event" predicates
	; This needs to happen before evaluating the monitored params so the
	; new-event predicates are set beforehand.
	(for-each set-new-event-status psi-monitored-events)

	; Evaluate the monitored params and set "changed" predicates accordingly
	(set! changed-params '())
	(for-each set-param-change-status psi-monitored-entities)
	;(format #t "\nchanged-params: ~a\n\n" changed-params)

	; Check for changed PAUs, just for highlighting in test output
	; (set! monitored-pau (list voice-width))
	(for-each
		(lambda (pau)
			(let* ((previous (hash-ref prev-value-table pau))
				(current (psi-get-number-value pau)))
					;(format #t "pau: ~a  prev: ~a  current: ~a\n" pau previous
					;    current)
					; if previous is #f, means it has not been set yet
					(if (equal? previous #f)
						(begin
							(hash-set! prev-value-table pau current)
							(set! previous current)))
					(if (not (equal? previous current))
						(begin
							(set! changed-pau (append changed-pau (list pau)))
							(hash-set! value-at-step-start pau current)
						))))
			monitored-pau)
			;(format #t "Changed PAU's: ~a\n" changed-pau)

	(if logging
		(if (or (not (null? changed-params))
				(not (null? detected-events))
				(not (null? changed-pau)))
			(let ((output-port (open-file "psilog.txt" "a")))
				(format output-port
					(string-append "-------------------------------------------"
						"-------------------------------- Loop ~a\n")
					psi-updater-loop-count)

				(format output-port
					(string-append
						"speech: ~a  new-face: ~a  "
						"pos-dialog: ~a  neg-dialog: ~a  "
						"\n\n"
						"arousal: ~a  pos-valence: ~a  neg-valence: ~a  "
						"agent power: ~a  "
						;"voice: ~a  "
						"\n\n")
					(highlight-display speech-giving-starts)
					(highlight-display new-face)
					(highlight-display positive-sentiment-dialog)
					(highlight-display negative-sentiment-dialog)
					(highlight-display arousal)
					(highlight-display pos-valence)
					(highlight-display neg-valence)
					(highlight-display power)
					;(highlight-display voice-width)
				)
				(close-output-port output-port))))

	; Grab and evaluate the interaction rules
	; todo: Could optimize by only calling rules containing the changed params
	(let ((rules (psi-get-interaction-rules)))
		(map psi-evaluate-interaction-rule rules)
	)

	; Have OpenPsi trigger emotion expression updates after events are detected
	; that impact modulator and sec variables
	; Trigger an expression update after n loop steps have occurred following an
	; event occurrence.
	(let ((event-at-loop-num
				(psi-get-number-value psi-event-at-loop-num-node)))
		;(format #t "\nevent-at-loop-num: ~a\n" event-at-loop-num)
		(if (= event-at-loop-num 0)
			; If a new event has been detected, set the loop count for it.
			(if (> (length detected-events) 0)
				(psi-set-value! psi-event-at-loop-num-node psi-updater-loop-count))

			; else, event at loop count is already set, so check if it's time
			; to fire off the expression callback
			(if (= (+ event-at-loop-num psi-expression-loop-delay)
			       psi-updater-loop-count)
				(begin
					(if (not (unspecified? psi-expression-callback))
						(apply psi-expression-callback '()))
					(psi-set-value! psi-event-at-loop-num-node 0)))))

	; Decay dynamic variable values toward their baselines
	(for-each
		(lambda (entity)
			(define new-value)
			(define baseline (psi-get-baseline-value entity))
			; Calculate difference between current and baseline value
			(define diff (- (psi-get-number-value entity) baseline))
			(set! diff (* decay-factor diff))
			(set! new-value (+ baseline diff))
			(psi-set-value! entity new-value))
		psi-modulators-and-secs
	)

	; Update prev-value-table entries for the changed (monitored) params
	(for-each 
		(lambda (param)
			(define value (hash-ref value-at-step-start param))
			(hash-set! prev-value-table param value))
		changed-params)

	; Update prev-value-table entries for the changed pau's
	;(This is just for log highlighint for testing)
	(for-each 
		(lambda (pau)
			(define value (hash-ref value-at-step-start pau))
			(hash-set! prev-value-table pau value))
		changed-pau)

	; Set detected events value to 0, since any particular event instance
	; should fire rules for only one step (could also do this at the beginning
	; of the step function)
	(for-each (lambda (event)
				(psi-set-value! event 0)
				(hash-set! prev-value-table event 0))
			detected-events)

	(psi-pause)
	; Single step below didn't seem to work well
	(if single-step
		(psi-updater-halt))
	(stv 1 1)

	;(set! psi-updater-loop-count (+ psi-updater-loop-count 1))

)


; --------------------------------------------------------------

(define-public (adjust-psi-var-level target strength trigger)
"
 Adjust psi-related variable based on triggered interaction rule

 Update openpsi parameter and variable values as a functon of the magnitude of
 change in the trigger entity, the current value of the target and the strength
 of the interaction rule. The current value of the target influences the change
 magnitude such that increases in value are larger when the current value is low
 and smaller when the current value is high. (And vica versa for decreases.)

 Currently the implementation assumes the values of openpsi parameters are
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
	(if verbose (begin
		(display "\n------------------------------------------------------\n")
		(format #t "adjust-psi-var-level   target: ~a   trigger: ~a    loop: ~a\n"
			target trigger psi-updater-loop-count)))
	(let* ((current-value (psi-get-number-value target))
	       (trigger-change (psi-get-change-magnitude trigger))
	      )

		; If current value is not a number (e.g., #f for not previously set),
		; set it to .5.
		(if (not (number? current-value))
			(set! current-value .5))

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

			; else strength is > .5 so set multiplier between 1 and some
			; specified max using the formula that god only knows how i came up
			; with but i think actually works.
			(let ((max psi-max-strength-multiplier))
				(set! strength-multiplier
					(- (+ (* (- (* 2 max) 2) strength) 2) max)))
		)

		(set! alpha (* trigger-change strength-multiplier dynamics-sensitivity))
		; make sure alpha is in [-1,1]
		(if (> alpha 0)
			(set! alpha (min alpha 1))
			(set! alpha (max alpha -1)))

		(if verbose
			(format #t "strength: ~a     change: ~a       alpha: ~a\n"
				strength trigger-change alpha))

		; Increasing slope increases the degree change at all levels
		(set! slope 10000)
		;(format #t "current value: ~a\n" current-value)
		; TODO: handle current value = #f (or not number in general)
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
		(if verbose
			(format #t "current value: ~a    setting new value: ~a\n"
				current-value
				new-value))

		(psi-set-value! target new-value)
	)
)

; Ultradian rhythm update function
; Updates value of var based on cos wave function
; Called via GroundedSchemaNodes in Implication rules
(define-public (psi-ultradian-update var Beta omega offset)
"
  var - psi variable as Atom
  Beta - amplitude as NumberNode
  omega - frequency in radians (iow frequency per 2*pi timesteps) as NumberNode
  offset - period offset as NumberNode
"
	(define val (psi-get-number-value var))
	(define ultradian-rhythm)

	; Convert NumberNodes to numbers
	(set! Beta (string->number (cog-name Beta)))
	(set! omega (string->number (cog-name omega)))
	(set! offset (string->number (cog-name offset)))

	(if val
		(begin
			(set! ultradian-rhythm (* Beta omega (cos
				(* omega (+ psi-updater-loop-count offset)))))
			(set! val (+ val ultradian-rhythm))
			(set! val (min (max 0 val) 1))
			(psi-set-value! var val)))

	;(format #t "~a: ~a\n" var val)
)

; Adjust openpsi variable by adding noise
; Called via GSN's in Implication rules
(define-public (psi-noise-update var width)
"
  var - psi variable as Atom
  width - width of the random number adjustement range as NumberNode. Noise to
    add will be in the range of [-width/2, width/2]
"
	(define val (psi-get-number-value var))
	(define noise)
	(set! width (string->number (cog-name width)))
	(set! noise (- (random width) (/ width 2)))
	(set! val (+ val noise))
	(set! val (min (max 0 val) 1))
	(psi-set-value! var val)
	;(format #t "post-noise ~a: ~a\n" var val)
)

; Return random offset based on cycle frequence
(define-public (get-random-cycle-offset omega)
	(random (/ (* 2 3.14) omega)))

; --------------------------------------------------------------
; Helper functions

(define (psi-get-interaction-rules)
	(cog-outgoing-set
			(cog-execute!
				(Get
					(Member
						(Variable "$rule")
						psi-interaction-rule)))))

; helper to identify (Eval (Predicate "psi-changed" (List . . .))) links
(define (psi-changed-eval? atom)
	(if (and (equal? (cog-type atom) 'EvaluationLink)
			 (or
				 (equal? (gar atom) (Predicate "psi-changed"))
				 (equal? (gar atom) (Predicate "psi-increased"))
				 (equal? (gar atom) (Predicate "psi-decreased"))))
		#t
		#f))

(define (psi-change-in? target)
	;(display "psi-change-in? target: \n")(display target)
	(let* ((previous (hash-ref prev-value-table target))
	       (current (psi-get-number-value target)))

		; If no previous value set, attempt to set it based on current
		(if (equal? previous #f)
			(hash-set! prev-value-table target current))

		;(format #t "previous value: ~a   current value: ~a\n" previous current)
		(if
			(and
				; current value is defined
				(not (equal? #f current))
				(not (equal? #f previous))
				(not (= previous current))
				(not (and (equal? previous #f)
				          (= current 0)))
			)
			#t
			#f
		)
	)
)

(define (psi-monitored-event? target)
	(member target psi-monitored-events))

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

		; Special handling for events. Moderate the magnitude change since the
		; monitored event values are binary, the change will always be 1.
		; Set event change magnitude to .5 for events.
		(if (psi-monitored-event? target)
			(set! return .5))

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
		(for-each 
			(lambda (sub-atom)
				(set! results
					(append (cog-filter-hypergraph pred? sub-atom) results))
			)
			(cog-outgoing-set atom)))
	results
)

; ----------------------------------------------------------------------
(define (psi-get-most-recent-ts event)
	(cog-get-state-value
		(List
			event
			psi-most-recent-occurrence-pred)))

; --------------------------------------------------------------
; Updater Loop Control
; --------------------------------------------------------------
; Todo: Perhaps will want to integrate this into the main OpenPsi loop, but OTOH
; we may want them running at different frequencies.

(define psi-updater-is-running #f)
(define psi-updater-loop-count 1)
(define continue-psi-updater-loop (stv 1 1))

(define-public (psi-updater-running?)
"
  Return #t if the openpsi dynamics updater loop is running, else return #f.
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
	(define pause-time 100000)
	(if slo-mo
		(set! pause-time 1000000))
	(usleep pause-time)
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

	(if (not psi-updater-is-running)
		(begin
			(psi-updater-init)
			(set! psi-updater-is-running #t)
			(cog-set-tv! updater-continue-pred (stv 1 1))
			(call-with-new-thread
				(lambda () (cog-evaluate! (loop-node)))))
	)
)

(define-public (psi-updater-halt)
"
  Tells the psi loop thread, that is started by running `(psi-update-run)`, to
  exit.
"
	(set! psi-updater-is-running #f)
	(cog-set-tv! updater-continue-pred (stv 0 1))
)
