; Basic demo of ForAll unification for arguments to ModusPonensRule

(define tv (stv 1 0.999))

(define is_axiom (PredicateNode "is_axiom" tv))
(define R (PredicateNode "R" tv))
(define x001 (VariableNode "x001" tv))
(define fact_6 (ConceptNode "fact_6" tv))
(define fact_42 (ConceptNode "fact_42" tv))

(ForAllLink tv (ListLink x001)
    (EvaluationLink
        is_axiom
        (ListLink x001)
    )
)

; Breaks BC
(define target
    (AndLink
        (EvaluationLink
            is_axiom
            fact_6
        )
        (EvaluationLink
            is_axiom
            fact_42
        )
    )
)
