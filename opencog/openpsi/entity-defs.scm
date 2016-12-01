;
; entity-defs.scm
;
; OpenPsi-related entity definitions for use in
; the interaction dynamics rules.
;
;
; --------------------------------------------------------------

; EVENT PREDICATES
; Event/Stimilus Predicates are assumed to get evaluate to 1 for a single
; psi-loop step and only for a single psi-loop step whenever a particular
; instance of the event is detected.

; ------------------------------------------------------------------
; THIS IS TEMPORARY FOR DEVELOPMENT/TESTING - PAU'S WILL NOT LIVE IN THIS DICTORY
; PAU Predicates
; Actually these will be defined somewhere else in the system
; PAU(Physiological Action Unit) is a kind of physiological command to the
; robot.
(define pau-prefix-str "PAU: ")
(define (create-pau name initial-value)
	(define pau
		(Predicate (string-append pau-prefix-str name)))
	(Inheritance
		pau
		(Concept "PAU"))
	(psi-set-value! pau initial-value)
	;(hash-set! prev-value-table pau initial-value)
	pau)

(define voice-width
	(create-pau "voice width" .2))


; --------------------------------------------------------------
; Shortcuts for dev use

;(define r1 speech->power)
;(define r2 power->voice)

(define-public p agent-state-power)
(define-public a arousal)

(define voice voice-width)
