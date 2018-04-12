; ----------
; ECAN related configuration

; Update some of the parameters
(State (Concept "AF_RENT_FREQUENCY") (Number 0.5))
(State (Concept "MAX_SPREAD_PERCENTAGE") (Number 0.8))

(define default-stimulus 150)

; ----------
(define-public (ghost-stimulate . ATOMS)
"
  Stimulate the given list of atoms with the default stimulus.
"
  (map (lambda (a) (cog-stimulate a default-stimulus)) ATOMS)
)

(define-public (ghost-stimulate-words . WORDS)
"
  Stimulate the given list of words (as strings) with the default stimulus.
"
  ; 'ghost-word-seq' is shared among the rules with word-related pattern
  ; This is mainly to make sure the rules with only a wildcard in the pattern
  ; will also get some non-zero STI.
  ; TODO: Find some better representation for that
  (ghost-stimulate ghost-word-seq)

  (map (lambda (w) (cog-stimulate (Word w) default-stimulus)) WORDS)
)

; ----------
(define stimulate-timer-elapsed-time 0)
(define (ghost-stimulate-timer)
"
  Stimulate the timer predicate, so that the rules having time-related
  predicates will likely have some non-zero STI.

  Currently the stimulus will be proportional to the elapsed time (sec)
  since last time it's called.
"
  (if (> stimulate-timer-elapsed-time 0)
    (cog-stimulate (Concept "timer-predicate")
      (* 10 (- (current-time) stimulate-timer-elapsed-time))))

  (set! stimulate-timer-elapsed-time (current-time))
)
