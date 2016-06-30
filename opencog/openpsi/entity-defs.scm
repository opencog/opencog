;
; entity-defs.scm
;
; OpenPsi-related modulator, parameter, and other entity definitions for use in
; the interaction dynamics rules.

(load "modulator.scm")
(load "sec.scm")

; --------------------------------------------------------------

; Create Modulators
(define arousal (psi-create-modulator "arousal" .5))

; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))

; Create SECs
(define power (psi-create-sec "Power"))

; Create Stimulus-SEC Associations
(define agent-state-power
	(psi-create-stimulus-sec agent-state power .5))


; EVENT PREDICATES
(define speech (Predicate "speech-giving-starts"))
(Evaluation speech (List) (stv 0 1))
;(hash-set! prev-value-table speech 0.0)

; PAU PREDICATES
; Actually these will be defined somewhere else in the system
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

