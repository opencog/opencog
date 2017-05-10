(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define w (cons 'word "drink"))
(define p (cons 'phrase "John Smith"))
(define c (cons 'concept "play"))
(define neg (negation (list w p c)))

; Check the EvaluationLink created
(define test-negation-result
    (let ((c (car (cdr neg))))
        (and (eq? (cog-type c) 'EvaluationLink)
             (equal? (gar c) (GroundedPredicateNode "scm: chatlang-negation?"))
             (equal? (gdr c) (ListLink (WordNode "drink")
                                       (ListLink (WordNode "John")
                                                 (WordNode "Smith"))
                                       (ConceptNode "play"))))))
