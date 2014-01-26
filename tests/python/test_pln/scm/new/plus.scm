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

;(EvaluationLink (stv 1 1)
;    (PredicateNode "+")
;    (ListLink
;        (NumberNode "1")
;        (NumberNode "1")
;        (NumberNode "2")
;    )
;)

(define target (EvaluationLink (ConceptNode "WIN")))
(ForAllLink (stv 1 1)
    (ListLink (VariableNode "TheAmazingResult"))
    (ImplicationLink sum target)
)
