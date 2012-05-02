(define sum
(EvaluationLink
    (PredicateNode "+")
    (ListLink
        (NumberNode "1")
        (NumberNode "1")
        ;(NumberNode "2")
        (VariableNode "TheAmazingResult")
    )
)
)

(define target (EvaluationLink (ConceptNode "WIN")))
(ImplicationLink sum target (stv 1 1))
