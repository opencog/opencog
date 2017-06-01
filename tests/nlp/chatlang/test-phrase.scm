(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define phrase (phrase "John Smith"))

; Just want to see if the ReferenceLink is there...
(define test-phrase-result
    (equal? 2 (length
        (filter (lambda (x)
            (and (eq? (cog-type x) 'ReferenceLink)
                 (or (equal? (gdr x) (WordNode "John"))
                     (equal? (gdr x) (WordNode "Smith")))))
        (cdr phrase)))))
