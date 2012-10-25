(EvaluationLink (stv 1 1)
    (PredicateNode "agent_location")
    (ListLink
        (ConceptNode "at (90, 90, 98)" (stv 1 1))
    )
)

(ForAllLink (stv 1 1)
    (ListLink)
    (ImplicationLink
        (AndLink
            (EvaluationLink
                (PredicateNode "neighbor")
                (ListLink
                    (VariableNode "at1")
                    (VariableNode "at2")
                )
            )
            (EvaluationLink
                (PredicateNode "agent_location")
                (ListLink
                    (VariableNode "at2")
                )
            )
        )
        (EvaluationLink
            (PredicateNode "agent_location")
            (ListLink
                (VariableNode "at1")
            )
        )
    )
)

(define target
    (EvaluationLink
       (PredicateNode "agent_location")
       (ListLink
           (ConceptNode "at (42, 73, 98)")
       )
    )
)