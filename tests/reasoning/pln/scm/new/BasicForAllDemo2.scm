; More advanced ForAllDemo (ironically), which puts the ImplicationLink in a ForAll as well as the premise.

(define tv (stv 1 0.999))

(define is_axiom (PredicateNode "is_axiom" tv))
(define R (PredicateNode "R" tv))
(define x001 (VariableNode "x001" tv))
(define x002 (VariableNode "x002" tv))
(define x003 (VariableNode "x003" tv))
(define fact_6 (ConceptNode "fact_6" tv))
(define fact_42 (ConceptNode "fact_42" tv))

(ForAllLink tv (ListLink x001)
    (EvaluationLink
        is_axiom
        (ListLink x001)
    )
)

; tackily use the result, so that FC will know what binding to use
(ForAllLink tv (ListLink x002)
    (ImplicationLink tv
        (EvaluationLink
            is_axiom
            x002
        )
        (EvaluationLink
            R
            (ListLink fact_42)
        )
    )
)

(define target
    (ImplicationLink
        (EvaluationLink
            is_axiom
            fact_6
        )
        (EvaluationLink
            R
            (ListLink fact_42)
        )
    )
)

(define target
    (EvaluationLink
        R
        (ListLink fact_42)
    )
)
