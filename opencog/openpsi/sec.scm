;
; sec.scm
;
; Stimulus Evaluation Check (SEC) entity from Process Component Model theory
;
; SEC are associated with a particular stimulus or the agent state. Thus, they
; do not have values on their own but only in association with a stimulus.

(define (psi-create-sec name)
    (define sec
        (Concept (string-append psi-prefix-str name)))
    (Inheritance
        sec
        (Concept (string-append psi-prefix-str "SEC")))
    sec)

(define (psi-create-stimulus-sec stimulus sec initial-value)
"
  Create a stimulus-SEC association and assign its initial value.
  A stimulus-SEC takes the form (List stimulus SEC) and its current value is
  stored in a StateLink:
      (StateLinke (List stimulus SEC) (Number x))
  The value is assumed to be in [0,1].
"
	(define stimulus-sec
		(List
	        stimulus
	        sec))
    (psi-set-value! stimulus-sec (Number initial-value))
    stimulus-sec)


; =============================================================================
; CREATE SECs

(define novelty (psi-create-sec "novelty"))
(define goal-relevance (psi-create-sec "goal-relevance"))
(define pleasantness (psi-create-sec "pleasantness"))
(define outcome-probability (psi-create-sec "outcome-probability"))
(define surprise (psi-create-sec "surprise"))
(define agent-and-intention (psi-create-sec "agent-and-intention"))
(define control (psi-create-sec "control"))
(define power (psi-create-sec "power"))
(define adjustment (psi-create-sec "adjustment"))
(define standards (psi-create-sec "standards"))

; --------------------------------------------------------------

; Agent State
(define agent-state (Concept (string-append psi-prefix-str "agent-state")))

; --------------------------------------------------------------

; Create Stimulus-SEC Associations
(define agent-state-power
	(psi-create-stimulus-sec agent-state power .5))


