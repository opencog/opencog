(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; concept is not define-public, so we have to load terms.scm to test it.
(load "../../../opencog/nlp/chatlang/terms.scm")

(define concept (concept "drink"))

; Check the EvaluationLink created
(define test-concept-result
    (let ((c (car (cdr concept))))
        (and (eq? (cog-type c) 'EvaluationLink)
             (equal? (gar c) (GroundedPredicateNode "scm: chatlang-concept?"))
             (equal? (gadr c) (ConceptNode "drink")))))
