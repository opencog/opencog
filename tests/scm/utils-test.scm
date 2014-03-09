;
; Test utilities

(define counter (ConceptNode "asdf"))

(define (test-incr-cnt) (cog-atom-incr counter 1)
