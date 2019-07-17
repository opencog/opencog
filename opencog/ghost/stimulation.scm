
(use-modules (opencog) (opencog nlp) (opencog nlp chatbot))

; ----------
; ECAN related configuration

; Update some of the parameters
(State (Concept "AF_RENT_FREQUENCY") (Number 0.5))
(State (Concept "MAX_SPREAD_PERCENTAGE") (Number 0.8))
(State (Concept "HEBBIAN_MAX_ALLOCATION_PERCENTAGE") (Number 1))

(define default-stimulus 150)

; Stimulate words when doing 'nlp-parse'
(nlp-start-stimulation default-stimulus)

; ----------
(define-public (ghost-set-default-stimulus STIMULUS)
"
  Change the default stimulus to the given value.
"
  (if (number? STIMULUS)
    (set! default-stimulus STIMULUS)
    (cog-logger-warn ghost-logger "Stimulus should be a number!"))
)


; Add a hook to be run when a word is perceived
; 'ghost-word-seq' is shared among the rules with word-related pattern
; This is mainly to make sure the rules with only a wildcard in the pattern
; will also get some non-zero STI.
; TODO: Find some better representation for that
(add-hook! (perceive-word-hook)
  (lambda () (cog-stimulate (ghost-word-seq-pred) (/ default-stimulus 2))))
