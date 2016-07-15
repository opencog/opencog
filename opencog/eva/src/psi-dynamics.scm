;
; psi-dynamics.scm
;
; OpenPsi dynamics control for Hanson robot

(use-modules (opencog) (opencog exec) (opencog openpsi))

(load "express.scm") ; For random pos and neg expressions

; Param setting
(define valence-activation-level .4)

(define psi-verbose #t)
(define no-blender #f)

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
	;(if psi-verbose (display "psi-dynamics expression callback called\n"))

	; For now we are doing something simple - do a random positive or negative
	; expression based on valence and arousal.
	; Later arousal will be used to modulate intensity of the expression
	; Todo: How to handle when both pos and neg valence are high ? Expressing
	; both for now, which may or may not be a good thing, but probably
	; interesting nonetheless.
	(if (>= pos-valence valence-activation-level)
		;(do-catch do-random-positive-expression))
		(if no-blender
			(if psi-verbose (display "doing random positive expression\n"))
			(do-random-positive-expression)))
	(if (>= neg-valence valence-activation-level)
		;(do-catch do-random-negative-expression))
		(if no-blender
			(if psi-verbose (display "doing random negative expression\n"))
			(do-random-negative-expression)))

	;(set! verbose prev-verbose)
)

; Register the expression callback function with OpenPsi
(psi-set-expression-callback! psi-expression-callback)

; ------------------------------------------------------------------
; Functions to initiate random positive and negative epxerssions

(define (do-random-positive-expression)
   (cog-execute!
      (PutLink (DefinedSchemaNode "Pick random expression")
         (ConceptNode "positive"))))

(define (do-random-negative-expression)
   (cog-execute!
      (PutLink (DefinedSchemaNode "Pick random expression")
         (ConceptNode "negative"))))

;(format #t "do_random_postive_expression: ~a\n" (do_random_positive_expression))

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
; OpenPsi Dynamics Interaction Rules

; Todo: move to own file probably

; The following change-predicate types have been defined in
; opencog/opencog/openpsi/interaction-rule.scm:
;(define changed "changed")
;(define increased "increased")
;(define decreased "decreased")

(define power->voice
	(psi-create-interaction-rule agent-state-power changed voice-width 1))




; ------------------------------------------------------------------
; Run the dyanmics updater loop. Eventually this will be part of the main
; OpenPsi loop.
(psi-updater-run)




