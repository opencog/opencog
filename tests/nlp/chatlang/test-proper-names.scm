(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define pn (proper-names "Hanson" "Robotics"))

; Just want to see if the EvaluationLink and ReferenceLink are there...
(define test-result
    (equal? 3 (length (filter (lambda (x)
        (or (and (eq? (cog-type x) 'EvaluationLink)
                 (equal? (gar x) (LinkGrammarRelationshipNode "G")))
            (and (eq? (cog-type x) 'ReferenceLink)
                 (equal? (gdr x) (Word "Hanson")))
            (and (eq? (cog-type x) 'ReferenceLink)
                 (equal? (gdr x) (Word "Robotics")))))
        (cdr pn)))))
