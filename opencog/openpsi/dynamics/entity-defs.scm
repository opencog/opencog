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




;;; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))


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

