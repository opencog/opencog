(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define concept (concept "drink"))

; Just want to see if the ReferenceLink connecting
; the variable and the concept is there...
(define test-concept-result
    (not (null? (filter (lambda (x)
        (and (eq? (cog-type x) 'ReferenceLink)
             (equal? (gdr x) (ConceptNode "drink"))))
        (cdr concept)))))
