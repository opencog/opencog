(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define ms (main-subj "Ben"))

; Just want to see if the EvaluationLinks and LemmaLink are there...
(define test-result
    (equal? 3 (length (filter (lambda (x)
        (or (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (LinkGrammarRelationshipNode "WV")))
            (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (DefinedLinguisticRelationshipNode "_subj")))
            (and (eq? (cog-type x) 'LemmaLink)
                 (equal? (gdr x) (Word "Ben")))))
        (cdr ms)))))
