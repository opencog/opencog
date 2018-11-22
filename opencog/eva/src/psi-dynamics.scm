;
; psi-dynamics.scm
;
; OpenPsi dynamics control for Hanson robot

(use-modules (opencog) (opencog exec) (opencog openpsi))
(use-modules (opencog openpsi dynamics))

; needed for nlp parsing
(use-modules (opencog nlp) (opencog nlp chatbot) (opencog nlp chatbot-psi))

(use-modules (opencog eva-behavior)) ; for get-input-sent-node def


; ------------------------------------------------------------------
; Default modulators
(define openpsi-modulators (psi-default-modulator-alist))

(define arousal (assoc-ref openpsi-modulators "arousal"))
(define neg-valence (assoc-ref openpsi-modulators "neg-valence"))
(define pos-valence (assoc-ref openpsi-modulators "pos-valence"))
(define resolution-level (assoc-ref openpsi-modulators "resolution-level"))
(define goal-directedness (assoc-ref openpsi-modulators "goal-directedness"))

; Default SEC
(define openpsi-secs (psi-default-sec-alist))

(define power (assoc-ref openpsi-secs "power"))

; ------------------------------------------------------------------
; Param setting
(define valence-activation-level .5)

(define psi-verbose #t)

(define single-dimension-valence #t)

(define prev-value-node (Concept "previous value"))
(define current-sentence-node (Concept "current sentence"))

; Function called by OpenPsi when it believes expression command updates should
; occur based on event detection and subsequent OpenPsi variable updating
(define (psi-expression-callback)
	;(define arousal-value (psi-get-arousal))
	;(define pos-valence-value (psi-get-pos-valence))
	;(define neg-valence-value (psi-get-neg-valence))
	;(if psi-verbose (display "psi-dynamics expression callback called\n"))

	(define dominant-emotion (psi-get-current-emotion))
	(define emotion-value (psi-get-number-value dominant-emotion))
	;(show-emotion (cog-name dominant-emotion) emotion-value)
	(if (equal? dominant-emotion psi-happy)
		(be-happy emotion-value))
	(if (equal? dominant-emotion psi-sad)
		(be-sad emotion-value))
	(if (equal? dominant-emotion psi-angry)
		(be-irritated emotion-value))
	(if (equal? dominant-emotion psi-relaxed)
		(be-comprehending emotion-value))
)

; Register the expression callback function with OpenPsi
(psi-set-expression-callback! psi-expression-callback)

; ------------------------------------------------------------------
; Functions to initiate random positive and negative expressions

; XXX FIXME -- this is hacky -- and has multiple design flaws.
; These are:
;
; * This should not use cog-evaluate! to force execution. Instead,
;   this should be placed in the action part of an open-psi rule.
;   Then, when that psi-rule triggers, the action will automatically
;   triggered, and the actin will be performed. No cog-execute! is
;   needed, because it will happen automatically.
;
; * This should used the defined predicates in express.scm, which
;   will automatically provide an appropriate, configurable, random
;   duration for the expression, instead of the hard-coded 8 seconds
;   below.
;
; * There are a hell of a lot more valencies than just "positive"
;   and "negative". Take a look at cfg-sophia.scm to see some of them:
;   these include: bored, sleeping, aroused, listening (attentive),
;   speaking (active). A full list of nine valencies are given in
;   the `README-affects.md` in the base directory.
(define (do-random-positive-expression intensity)
	 (cog-evaluate!
		 (Put (DefinedPredicate "Show facial expression")
			 (ListLink
				 (PutLink (DefinedSchemaNode "Pick random expression")
					 (ConceptNode "positive"))
				 (Number 8) (Number intensity)))))

(define (do-random-negative-expression intensity)
	 (cog-evaluate!
		 (Put (DefinedPredicate "Show facial expression")
			 (ListLink
				 (PutLink (DefinedSchemaNode "Pick random expression")
					 (ConceptNode "frustrated"))
				 (Number 8) (Number intensity)))))

(define (show-emotion emotion intensity)
	(cog-evaluate! (Put (DefinedPredicate "Show facial expression")
		(ListLink (Concept emotion) (Number 8) (Number intensity)))))


(define (be-happy intensity)
	;(display "in (be-happy)\n")
	(cog-evaluate! (Put (DefinedPredicate "Show facial expression")
		(ListLink (Concept "happy") (Number 8) (Number intensity)))))

(define (be-sad intensity)
	;(display "in (be-sad)\n")
	(cog-evaluate! (Put (DefinedPredicate "Show facial expression")
		(ListLink (Concept "sad") (Number 8) (Number intensity)))))

(define (be-irritated intensity)
	;(display "in (be-irritated)\n")
	(cog-evaluate! (Put (DefinedPredicate "Show facial expression")
		(ListLink (Concept "irritated") (Number 8) (Number intensity)))))

(define (be-comprehending intensity)
	;(display "in (be-comprehending)\n")
	(cog-evaluate! (Put (DefinedPredicate "Show facial expression")
		(ListLink (Concept "comprehending") (Number 8) (Number intensity)))))

; Temp error catching for when blender not running
(define (do-catch function . params)
	(catch #t
	  (lambda ()
		(apply function params))
	(lambda (key . parameters)
		(format (current-error-port)
				  "\nUncaught throw to '~a: ~a\n" key parameters)
		)
	)
)

; ------------------------------------------------------------------
; Create Monitored Events
;-------------------------------------------------------------------
(define new-face (psi-create-monitored-event "new-face"))
(define speech-giving-starts
	(psi-create-monitored-event "speech-giving-starts"))
(define positive-sentiment-dialog
	(psi-create-monitored-event "positive-sentiment-dialog"))
(define negative-sentiment-dialog
	(psi-create-monitored-event "negative-sentiment-dialog"))


; Loud Noise
(define loud-noise-event
	(psi-create-monitored-event "loud-noise"))

; Callback for loud noise detected
(use-modules (opencog eva-model))
(define loud-noise (DefinedPredicate "Heard Loud Sound?"))
(define new-loud-noise? #f) ; indicates a loud noise just occurred
(define (psi-check-for-loud-noise)
	(if (equal? (cog-evaluate! loud-noise)
				(stv 1 1))
		(psi-set-event-occurrence! loud-noise-event)))

; Register the callback with the openpsi dynamics updater
(psi-set-event-callback! psi-check-for-loud-noise)

; Loud noise rules
(define loud-noise->arousal
	(psi-create-interaction-rule loud-noise-event "increased" arousal .9))
(define loud-noise->neg-valence
	(psi-create-interaction-rule loud-noise-event "increased" neg-valence .7))


; Room Luminance
; Room got bright and room got dark
(define room-got-bright-event (psi-create-monitored-event "room-got-bright"))
(define room-got-dark-event (psi-create-monitored-event "room-got-dark"))

(define room-was-bright #f)
(define room-was-dark #f)
(define (psi-detect-room-got-bright)
	(if (room-is-bright?)
		(if (not room-was-bright)
			(begin
				(psi-set-event-occurrence! room-got-bright-event)
				(set! room-was-bright #t)
				; TODO Replace with openpsi-dynamics logger when it is available.
				;(if verbose (display "**** Room got bright *****\n"))
			)
		)
		(if room-was-bright
			(set! room-was-bright #f))
	)
)

(define (psi-detect-room-got-dark)
	(if (room-is-dark?)
		(if (not room-was-dark)
			(begin
				(psi-set-event-occurrence! room-got-dark-event)
				(set! room-was-dark #t)
				; TODO Replace with openpsi-dynamics logger when it is available.
				;(if verbose (display "**** Room got dark *****\n"))
			)
		)
		(if room-was-dark
			(set! room-was-dark #f))
	)
)

(psi-set-event-callback! psi-detect-room-got-bright)
(psi-set-event-callback! psi-detect-room-got-dark)

; Luminance rules
(define room-got-bright->arousal
	(psi-create-interaction-rule room-got-bright-event "changed" arousal .5))

(define room-got-dark->arousal
	(psi-create-interaction-rule room-got-dark-event "changed" arousal -.4))

(define (get-luminance-value)
    (define result (cog-outgoing-set (cog-execute!
        (Get (State (Anchor "luminance") (Variable "$x"))))))
    (if (not (null? result))
		(set! result (cog-number (car result)))
		(set! result #f))
    ;(format #t "~a\n" result)
    result
)

(define (room-is-bright?)
	(define val (get-luminance-value))
	(define return)
	(if val
		(if (> val 50)
			(set! return #t)
			(set! return #f) )
		; no val set for luminance value
		(set! return room-was-bright)
	)
	;(format #t "~a\n" return)
	return
)

(define (room-is-dark?)
	(define val (get-luminance-value))
	(define return)
	(if val
		(if (< val 20)
			(set! return #t)
			(set! return #f) )
		; no val set for luminance value
		(set! return room-was-bright)
	)
	;(format #t "~a\n" return)
	return
)

; ------------------------------------------------------------------
; Event detection callbacks

; Callback function for positive and negative chat sentiment detection
(define (psi-detect-dialog-sentiment)
	(define current-input (get-input-sent-node))
	(define previous-input (psi-get-value current-sentence-node))
	;(format #t "current-input: ~a\n" current-input)
	(if (and (not (equal? current-input '()))
			 (not (equal? current-input previous-input)))
		; We have a new input sentence
		(begin
			;(if psi-verbose (format #t "\n* New input sentence detected *\n"))
			;(format #t "previous-input: ~a   current-input: ~a\n"
			;    previous-input current-input)
			(StateLink current-sentence-node current-input)
			; Check for positive and/or negative sentimement
			; Sentence sentiment is put in the atomspace as
			;   (Inheritance (Sentence "blah") (Concept "Positive")) or "Negative"
			;   or "Neutral"
			(let ((inher-super (cog-chase-link 'InheritanceLink 'ConceptNode
					current-input)))
				;(format #t "inher-super: ~a\n" inher-super)
				(for-each (lambda (concept)
							;(if psi-verbose
							;    (format #t "Dialog sentiment detected: ~a\n"
							;        concept))
							(if (equal? concept (Concept "Positive"))
								(psi-set-event-occurrence!
									positive-sentiment-dialog))
							(if (equal? concept (Concept "Negative"))
								(psi-set-event-occurrence!
									negative-sentiment-dialog)))
						inher-super)))))

; Callback checks for both positive and negative sentiment
(psi-set-event-callback! psi-detect-dialog-sentiment)


; Create emotions
(define psi-happy (psi-create-emotion "happy"))
(define psi-sad (psi-create-emotion "sad"))
(define psi-angry (psi-create-emotion "angry"))
(define psi-relaxed (psi-create-emotion "relaxed"))
(define psi-excited (psi-create-emotion "excited"))
(define psi-tired (psi-create-emotion "tired"))

; Update emotion state values and current dominant emotion state
(define (psi-update-emotion-states)
	; Using 2-dimensional Wundt model of emotions based on arousal and valence
	; for now
	(define arousal (psi-get-arousal))
	(define pos-valence (psi-get-pos-valence))
	(define neg-valence (psi-get-neg-valence))

	; Multiplier for emotion level calculations
	(define emotionality-factor 1.2)
	(define e emotionality-factor)

	(define (limit value)
		(min value 1))

	(psi-set-value! psi-happy
		(limit (* pos-valence arousal e)))
	(psi-set-value! psi-angry
		(limit (* neg-valence arousal e)))
	(psi-set-value! psi-sad
		(limit (* neg-valence (- 1 arousal) e)))
	(psi-set-value! psi-relaxed
		(limit (* pos-valence (- 1 arousal) e)))

	; Alternative formulas
	;(psi-set-value! psi-happy
	;	(max (min (+ pos-valence (/ (- arousal .5) 2)) 1) 0) )
	;(psi-set-value! psi-angry
	;	(max (min (+ neg-valence (/ (- arousal .5) 2)) 1) 0) )
	;(psi-set-value! psi-sad
	;	(max (min (- neg-valence (/ (- arousal .5) 2)) 1) 0) )
	;(psi-set-value! psi-relaxed
	;	(max (min (- pos-valence (/ (- arousal .5) 2)) 1) 0) )

	(psi-set-current-emotion-state)

	; return atom for ExOutLink requirement
	(True)
)

; Current emotion state
(define-public psi-current-emotion-state
	(Concept (string-append psi-prefix-str "current emotion state")))

(define (psi-set-current-emotion-state)
	; Find the emotion with the highest level and set as current emotion
	(define emotions (psi-get-emotions))
	(define strongest-emotion)
	(define highest-emotion-value -1)
	(for-each
		(lambda (emotion)
			(define emotion-value (psi-get-number-value emotion))
			(if (> emotion-value highest-emotion-value)
				(begin
					(set! strongest-emotion emotion)
					(set! highest-emotion-value emotion-value))))
		emotions)
	(if (not (equal? (psi-get-value psi-current-emotion-state) strongest-emotion))
		(begin
			(StateLink psi-current-emotion-state strongest-emotion)
			;(format #t "Current emotion state: ~a\n" strongest-emotion)
		)
	)
)

; Returns the current emotion state of the agent as an atom of the form
; (ConceptNode "OpenPsi: happy")
(define-public (psi-get-current-emotion)
	(psi-get-value psi-current-emotion-state))

;
;-------------------------------------
; Internal vars to physiology mapping

; PAU's
;(define pau-prefix-str "PAU: ")
; temp change for compatibility with psi graphing
(define pau-prefix-str psi-prefix-str)
(define (create-pau name initial-value)
	(define pau
		(Concept (string-append pau-prefix-str name)))
	(Inheritance
		pau
		(Concept "PAU"))
	(psi-set-value! pau initial-value)
	;(hash-set! prev-value-table pau initial-value)
	pau)


;---------------------------------------------------
; Modulator and emotion cyclical rhythms and noise
;---------------------------------------------------

; default utradian rhythm and noise params
(define default_B .05)
(define default_w .02)
;(define default_offset (get-random-cycle-offset arousal_w))
(define default_noise .015)

; arousal ultradian rhythm params
(define arousal_B .05)
(define arousal_w .02)
(define arousal_offset (get-random-cycle-offset arousal_w))
(define arousal_noise .03)

; arousal rhythm rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-ultradian-update")
	(List arousal (Number arousal_B) (Number arousal_w) (Number arousal_offset)))

; arousal (stochastic) noise rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-noise-update")
	(List arousal (Number arousal_noise)))

; pos-valence rhythm rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-ultradian-update")
	(List pos-valence (Number default_B) (Number default_w)
		(Number (get-random-cycle-offset default_w))))

; pos-valence (stochastic) noise rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-noise-update")
	(List pos-valence (Number default_noise)))

; neg-valence rhythm rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-ultradian-update")
	(List neg-valence (Number default_B) (Number default_w)
		(Number (get-random-cycle-offset default_w))))

; neg-valence (stochastic) noise rule
(psi-create-general-rule (TrueLink)
	(GroundedSchemaNode "scm: psi-noise-update")
	(List neg-valence (Number default_noise)))


;=============================================================
; OpenPsi Dynamics Interaction Rules
;=============================================================

; The following change-predicate types have been defined in
; opencog/opencog/openpsi/interaction-rule.scm:
; "changed" "increased" "decreased"

;--------------------------------------
; Internal dynamic interactions rules

; pos valence up decreases neg valence
(psi-create-interaction-rule pos-valence "increased" neg-valence -.2)

; neg valence up decreases pos valence
(psi-create-interaction-rule neg-valence "increased" pos-valence -.2)

; power increases arousal
;(define power->arousal
;	(psi-create-interaction-rule power "changed" arousal .3))

; arousal decreases resolution
(psi-create-interaction-rule arousal "changed" resolution-level .5)

; arousal increases goal directedness
(psi-create-interaction-rule arousal "changed" goal-directedness .5)

; power decreases neg valence
(psi-create-interaction-rule power "changed" neg-valence -.2)

; arousal increases pos valence
(psi-create-interaction-rule arousal "increased" pos-valence .1)

; pos valence increases power
(psi-create-interaction-rule pos-valence "changed" power .2)

; ^ arousal >>>> pos valence
; pos valence >>> power
; power <<< neg valence


;-----------------------
; Event-based triggers

; User dialog sentiment
(define pos-sentiment->pos-valence
	(psi-create-interaction-rule positive-sentiment-dialog
		"increased" pos-valence .3))
(define pos-sentiment->neg-valence
	(psi-create-interaction-rule positive-sentiment-dialog
		"increased" neg-valence -.3))
(define neg-sentiment->neg-valence
	(psi-create-interaction-rule negative-sentiment-dialog
		"increased" neg-valence .3))
(define neg-sentiment->pos-valence
	(psi-create-interaction-rule negative-sentiment-dialog
		"increased" pos-valence -.3))

; Speech giving starts
(define speech->power
	(psi-create-interaction-rule speech-giving-starts "increased"
		power .5))

; New face
(define new-face->arousal
	(psi-create-interaction-rule new-face "increased" arousal .3))

; Voice width
(define voice-width (create-pau "voice width" .2))

; power increases voice-width
(define power->voice
	(psi-create-interaction-rule power "changed" voice-width .7))

; arousal decreases voice-width
(define arousal->voice
	(psi-create-interaction-rule arousal "changed" voice-width -.3))


; --------------------------------------------------------
; Rule to update emotion states each loop
(define psi-update-emotions-rule
	(psi-create-general-rule (TrueLink)
		(GroundedSchemaNode "scm: psi-update-emotion-states")
		(List)))


; ------------------------------------------------------------------
; Run the dyanmics updater loop. Eventually might be part of the main
; OpenPsi loop.
(psi-updater-run)


; ------------------------------------------------------------------
; Shortcuts for dev and testing purposes
; --------------------------------------------------------------
(define e psi-set-event-occurrence!)

(define halt psi-updater-halt)
(define h halt)
(define r psi-updater-run)
;(define r1 speech->power)
;(define r2 power->voice)
(define value psi-get-number-value)
(define rules psi-get-interaction-rules)

(define (psi-decrease-value target)
	(psi-set-value! target
		(max 0 (- (psi-get-number-value target) .1))))
(define (psi-increase-value target)
	(psi-set-value! target
		(min 1 (+ (psi-get-number-value target) .1))))

(define d psi-decrease-value)
(define i psi-increase-value)

(define (psi-set-pred-true target)
	(Evaluation target (List) (stv 1 1)))
(define (psi-set-pred-false target)
	(Evaluation target (List) (stv 0 1)))

(define t psi-set-pred-true)
(define f psi-set-pred-false)

(define nv neg-valence)
(define pv pos-valence)

(define (place-neg-dialog)
	(define sentence (SentenceNode (number->string (random 1000000000))))
	(State (Anchor "Chatbot: InputUtteranceSentence")  sentence)
	(Inheritance sentence (Concept "Negative")))

(define (place-pos-dialog)
	(define sentence (SentenceNode (number->string (random 1000000000))))
	(State (Anchor "Chatbot: InputUtteranceSentence")  sentence)
	(Inheritance sentence (Concept "Positive")))

(define (simulate-loud-noise)
	(cog-execute!
		(Put (State (AnchorNode "Decibel value") (Variable "$y")) (Number "100")))
	(sleep 1)
	(cog-execute! (Put (State (AnchorNode "Decibel value") (Variable "$y")) (Number "50")))
)
; Sudden sound change value seems to not be working anymore
;	(define sudden-sound-change (AnchorNode "Sudden sound change value"))
;	(call-with-new-thread
;		(lambda ()
;			(psi-set-value! sudden-sound-change 1)
;			(sleep 2)
;			(psi-set-value! sudden-sound-change 0))))

; Shortcuts
(define v voice-width)
(define p power)
(define a arousal)

(define n simulate-loud-noise)
(define ln simulate-loud-noise)

(define nd place-neg-dialog)   ; nd neg dialog
(define pd place-pos-dialog)   ; pd pos dialog

(define s speech-giving-starts)
(define pos positive-sentiment-dialog)
(define neg negative-sentiment-dialog)
(define nf new-face)
