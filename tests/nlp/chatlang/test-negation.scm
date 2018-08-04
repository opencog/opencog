(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; negation is not define-public, so we have to load terms.scm to test it.
(load "../../../opencog/nlp/chatlang/terms.scm")

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
                                       (PhraseNode "John Smith")
                                       (ConceptNode "play"))))))
