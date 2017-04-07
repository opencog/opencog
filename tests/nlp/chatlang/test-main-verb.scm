(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define mv (main-verb "read"))

; Just want to see if that EvaluationLink and LemmaLink are there...
(define test-main-verb-result
    (equal? 2 (length (filter (lambda (x)
        (or (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (LinkGrammarRelationshipNode "WV")))
            (and (eq? (cog-type x) 'LemmaLink)
                 (equal? (gdr x) (Word "read")))))
        (cdr mv)))))
