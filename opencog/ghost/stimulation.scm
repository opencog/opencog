; ----------
; ECAN related configuration

; Update some of the parameters
(State (Concept "AF_RENT_FREQUENCY") (Number 0.5))
(State (Concept "MAX_SPREAD_PERCENTAGE") (Number 0.8))

(define default-stimulus 150)

; ----------
(define-public (ghost-set-default-stimulus STIMULUS)
"
  Change the default stimulus to the given value.
"
  (if (number? STIMULUS)
    (set! default-stimulus STIMULUS)
    (cog-logger-warn ghost-logger "Stimulus should be a number!"))
)

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
(define timer-last-stimulated 0)
(define (ghost-stimulate-timer)
"
  Stimulate the timer predicate, so that the rules having time-related
  predicates will likely have some non-zero STI.

  Currently the stimulus will be proportional to the elapsed time (sec)
  since last time it's called.
"
  (if (> timer-last-stimulated 0)
    (cog-stimulate (Concept "timer-predicate")
      (* 10 (- (current-time) timer-last-stimulated))))

  (set! timer-last-stimulated (current-time))
)
