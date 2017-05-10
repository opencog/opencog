(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define concept (concept "drink"))

; Check the EvaluationLink created
(define test-concept-result
    (let ((c (car (cdr concept))))
        (and (eq? (cog-type c) 'EvaluationLink)
             (equal? (gar c) (GroundedPredicateNode "scm: chatlang-concept?"))
             (equal? (gdr (gdr c)) (ConceptNode "drink")))))
