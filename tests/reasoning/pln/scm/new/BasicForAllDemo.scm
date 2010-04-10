(define tv (stv 1 0.999))

(define tackily_implied (PredicateNode "tackily_implied" tv))
(define R (PredicateNode "R" tv))
(define x001 (VariableNode "x001" tv))
(define fact_6 (ConceptNode "fact_6" tv))
(define fact_42 (ConceptNode "fact_42" tv))

(ForAllLink tv (ListLink x001)
    (EvaluationLink
        tackily_implied
        (ListLink x001)
    )
)

; tackily use the result, so that FC will know what binding to use
(ImplicationLink tv
    (EvaluationLink
        tackily_implied
        (ListLink fact_6)
    )
    (EvaluationLink
        R
        (ListLink fact_42)
    )
)

(define target
    (EvaluationLink
        R
        (ListLink fact_42)
    )
)
