;
; entity-defs.scm
;
; OpenPsi-related entity definitions for use in
; the interaction dynamics rules.

; --------------------------------------------------------------


; EVENT PREDICATES
(define speech (Predicate "speech-giving-starts"))
;(Evaluation speech (List) (stv .5 1))
(psi-set-value! speech 0)


; PAU PREDICATES
; Actually these will be defined somewhere else in the system
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

