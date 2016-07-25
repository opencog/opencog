;
; psi-dynamics.scm
;
; OpenPsi dynamics control for Hanson robot

(use-modules (opencog) (opencog exec) (opencog openpsi) (opencog python))

; needed for nlp parsing
(use-modules (opencog nlp) (opencog nlp chatbot))

(load "express.scm") ; For random pos and neg expressions
(load "faces.scm")
(load "self-model.scm") ; For soma-state def
(load "orchestrate.scm") ; For DefinedPredicate "Show expression"

; Param setting
(define valence-activation-level .5)

(define psi-verbose #t)
(define no-blender #f)

(if no-blender
    (python-eval "execfile('/usr/local/share/opencog/python/atomic-dbg.py')"))


(define prev-value-node (Concept "previous value"))
(define current-sentence-node (Concept "current sentence"))

; Temporary call needed to load dynamics code while it's in dev phase
(load-openpsi-in-development)

; Function called by OpenPsi when it believes expression command updates should
; occur based on event detection and subsequent OpenPsi variable updating
(define (psi-expression-callback)
	(define arousal (psi-get-arousal))
	(define pos-valence (psi-get-pos-valence))
	(define neg-valence (psi-get-neg-valence))
	;(define prev-verbose verbose)
	;(set! verbose #t)
	(if psi-verbose (display "psi-dynamics expression callback called\n"))

	; For now we are doing something simple - do a random positive or negative
	; expression based on valence and arousal.
	; Later arousal will be used to modulate intensity of the expression
	; Todo: How to handle when both pos and neg valence are high ? Expressing
	; both for now, which may or may not be a good thing, but probably
	; interesting nonetheless.
	(if (>= pos-valence valence-activation-level)
	    (begin
            ;( (do-catch do-random-positive-expression))
            (if psi-verbose
                (display "psi-dynamics: doing random positive expression\n"))
            (if (not no-blender)
                (be-happy pos-valence)
                ;(do-random-positive-expression)
            )
        )
	)
	(if (>= neg-valence valence-activation-level)
	    (begin
            ;( (do-catch do-random-negative-expression))
            (if psi-verbose
                (display "psi-dynamics: doing random negative expression\n"))
            (if (not no-blender)
                (be-sad neg-valence)
                ;(do-random-negative-expression)
		    )
	    )
	)

	;(set! verbose prev-verbose)
)

; Register the expression callback function with OpenPsi
(psi-set-expression-callback! psi-expression-callback)

; ------------------------------------------------------------------
; Functions to initiate random positive and negative epxerssions

(define (do-random-positive-expression intensity)
    (define expression
       (cog-execute!
          (PutLink (DefinedSchemaNode "Pick random expression")
             (ConceptNode "positive"))))
     (cog-evaluate! (Put (DefinedPredicate "Show expression")
         (ListLink expression (Number 8) (Number intensity)))))

(define (do-random-negative-expression intensity)
    (define expression
       (cog-execute!
          (PutLink (DefinedSchemaNode "Pick random expression")
             (ConceptNode "frustrated"))))
     (cog-evaluate! (Put (DefinedPredicate "Show expression")
         (ListLink expression (Number 8) (Number intensity)))))

(define (be-happy intensity)
    (display "in (be-happy)\n")
    (cog-evaluate! (Put (DefinedPredicate "Show expression")
        (ListLink (Concept "happy") (Number 8) (Number intensity)))))

(define (be-sad intensity)
    (display "in (be-sad)\n")
    (cog-evaluate! (Put (DefinedPredicate "Show expression")
        (ListLink (Concept "sad") (Number 8) (Number intensity)))))

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
(define new-face (psi-create-monitored-event "new-face"))
(define speech-giving-starts
	(psi-create-monitored-event "speech-giving-starts"))
(define positive-sentiment-dialog
	(psi-create-monitored-event "positive-sentiment-dialog"))
(define negative-sentiment-dialog
	(psi-create-monitored-event "negative-sentiment-dialog"))


; ------------------------------------------------------------------
; Event detection callbacks

(define no-input-utterance (ConceptNode "Chatbot: NoInputUtterance"))

; Callback functions for positive and negative chat sentiment detection
(define (psi-detect-dialog-sentiment)
	(define current-input (get-input-sent-node))
	(define previous-input (psi-get-value current-sentence-node))
	;(format #t "current-input: ~a\n" current-input)
	(if (and (not (equal? current-input no-input-utterance))
	         (not (equal? current-input previous-input)))
		; We have a new input sentence
		(begin
            (format #t "\n*** New input sentance detected *** loop ~a\n"
                psi-updater-loop-count)
            (format #t "previous-input: ~a   current-input: ~a\n"
                previous-input current-input)
            (StateLink current-sentence-node current-input)
            ; Check for positive and/or negative sentimement
            ; Sentence sentiment is put in the atomspace as
            ;   (Inheritance (Sentence "blah") (Concept "Positive")) or "Negative"
            ;   or "Neutral"
            (let ((inher-super (cog-chase-link 'InheritanceLink 'ConceptNode
                    current-input)))
                (format #t "inher-super: ~a\n" inher-super)
                (for-each (lambda (concept)
                            (format #t "concept: ~a\n" concept)
                            (if (equal? concept (Concept "Positive"))
                       		    (psi-set-event-occurrence!
                       		        positive-sentiment-dialog))
                            (if (equal? concept (Concept "Negative"))
                                (psi-set-event-occurrence!
                                    negative-sentiment-dialog)))
                        inher-super)))))

; Callback checks for both positive and negative sentiment
(psi-set-event-callback! psi-detect-dialog-sentiment)

; ------------------------------------------------------------------
; OpenPsi Dynamics Interaction Rules

; Todo: move to own file probably

; The following change-predicate types have been defined in
; opencog/opencog/openpsi/interaction-rule.scm:
;(define changed "changed")
;(define increased "increased")
;(define decreased "decreased")

(define pos-sentiment->pos-valence
	(psi-create-interaction-rule positive-sentiment-dialog
		increased pos-valence .3))
(define pos-sentiment->neg-valence
	(psi-create-interaction-rule positive-sentiment-dialog
		increased neg-valence -.3))
(define neg-sentiment->neg-valence
	(psi-create-interaction-rule negative-sentiment-dialog
		increased neg-valence .3))
(define neg-sentiment->pos-valence
	(psi-create-interaction-rule negative-sentiment-dialog
		increased pos-valence -.3))

(define power->voice
	(psi-create-interaction-rule agent-state-power changed voice-width 1))

(define speech->power
    (psi-create-interaction-rule speech-giving-starts increased
        agent-state-power .5))

(define new-face->arousal
    (psi-create-interaction-rule new-face increased arousal .3))


; ------------------------------------------------------------------
; Run the dyanmics updater loop. Eventually this will be part of the main
; OpenPsi loop.
(psi-updater-run)


; ------------------------------------------------------------------
; Shortcuts for dev and testing purposes
(define s speech-giving-starts)
(define pos positive-sentiment-dialog)
(define neg negative-sentiment-dialog)
(define nf new-face)

(define (place-neg-dialog)
	(define sentence (SentenceNode (number->string (random 1000000000))))
	(State (Anchor "Chatbot: InputUtteranceSentence")  sentence)
	(Inheritance sentence (Concept "Negative")))

(define (place-pos-dialog)
	(define sentence (SentenceNode (number->string (random 1000000000))))
	(State (Anchor "Chatbot: InputUtteranceSentence")  sentence)
	(Inheritance sentence (Concept "Positive")))

(define pos place-pos-dialog)
(define neg place-neg-dialog)


; ---------------------------------------------------------------------
; for testing/dev when chatbot-psi files are not already loaded
(if (not (defined? 'get-input-sent-node))
    (begin
        (display "Loading support files for testing\n")
        (load "../../opencog/opencog/nlp/chatbot-psi/utils.scm")
        (load "../../opencog/opencog/nlp/chatbot-psi/states.scm")
        (load "../../opencog/opencog/nlp/chatbot-eva/imperative-rules.scm"))
        (python-eval "execfile('atomic.py')"))


