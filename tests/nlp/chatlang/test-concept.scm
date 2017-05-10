(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define concept (concept "drink"))

; Just want to see if the EvaluationLink is there...
(define test-concept-result
    (let ((c (car (cdr concept))))
        (and (eq? (cog-type c) 'EvaluationLink)
             (equal? (gar c) (GroundedPredicateNode "scm: chatlang-concept?"))
             (equal? (gdr (gdr c)) (ConceptNode "drink")))))
