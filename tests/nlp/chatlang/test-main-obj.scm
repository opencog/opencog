(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define mo (main-obj "apple"))

; Just want to see if the EvaluationLinks and LemmaLink are there...
(define test-main-obj-result
    (equal? 3 (length (filter (lambda (x)
        (or (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (LinkGrammarRelationshipNode "WV")))
            (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (DefinedLinguisticRelationshipNode "_obj")))
            (and (eq? (cog-type x) 'LemmaLink)
                 (equal? (gdr x) (Word "apple")))))
        (cdr mo)))))
