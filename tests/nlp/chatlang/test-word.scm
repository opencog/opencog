(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define word (word "mint"))

; Just want to see if the ReferenceLink is there...
(define test-result
    (not (null? (filter (lambda (x)
        (and (eq? (cog-type x) 'ReferenceLink)
             (equal? (gdr x) (WordNode "mint"))))
        (cdr word)))))
