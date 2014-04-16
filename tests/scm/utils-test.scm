;
; Test utilities

; test count truth value incrementation
(define counter (ConceptNode "asdf"))
(define (test-incr-cnt) (cog-atom-incr counter 1))

; test cog-get-partner
(define partner (ConceptNode "partner"))
(define pare (ListLink partner counter))
(define recount (cog-get-partner pare partner))

; test cog-pred-get-partner
(define evl (EvaluationLink (WordNode "asdf") pare))
(define rrcnt (cog-pred-get-partner evl partner))

; test cog-get-link
(define ref (ReferenceLink (ConceptNode "asdf") (WordNode "pqrs")))
(define wref (car (cog-get-link 'ReferenceLink 'ConceptNode (WordNode "pqrs"))))
