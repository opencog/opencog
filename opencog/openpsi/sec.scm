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