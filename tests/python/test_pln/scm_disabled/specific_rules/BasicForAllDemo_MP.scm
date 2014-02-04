; Basic demo of ForAll unification for arguments to ModusPonensRule

(define tv (stv 1 0.999))

(define is_axiom (PredicateNode "is_axiom" tv))
(define R (PredicateNode "R" tv))
(define x001 (VariableNode "x001"))
(define fact_6 (ConceptNode "fact_6" tv))
(define fact_42 (ConceptNode "fact_42" tv))

(ForAllLink tv (ListLink x001)
    (EvaluationLink
        is_axiom
        (ListLink x001)
    )
)

; tackily use the result, so that FC will know what binding to use
(ImplicationLink tv
    (EvaluationLink
        is_axiom
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

; Simpler target but it makes BC crash...
;(define target
;    (AndLink
;        (EvaluationLink
;            is_axiom
;            fact_6
;        )
;        (EvaluationLink
;            is_axiom
;            fact_42
;        )
;    )
;)

(EvaluationLink (PredicateNode "query") (ListLink target))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "ModusPonensRule"))) ; also needs some quantifier rules that aren't done yet

