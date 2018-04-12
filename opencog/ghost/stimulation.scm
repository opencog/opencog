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
  Stimulate the given list of words with the default stimulus.
"
  (map (lambda (w) (cog-stimulate (Word w) default-stimulus)) WORDS)
)
