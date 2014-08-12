(EvaluationLink (PredicateNode "inputs")
    (ListLink
        (SatisfyingSetLink (stv .99 .99)
	        (VariableNode "$x")
	        (EvaluationLink
	            (PredicateNode "smokes")
	            (ListLink
	                (VariableNode "$x")
	            )
	        )
	    )
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "SatisfyingSetToConceptRule")
	)
)

(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs")
	(ListLink
	     (AndLink
	        (EvaluationLink
	            (PredicateNode "ConceptToPredicate")
	            (ListLink
	                (ConceptNode "smoke")
	                (PredicateNode "smoke")
	            )
	        )
	        (InheritanceLink
	            (ConceptNode "smokes")
	            (SatisfyingSetLink (stv .99 .99)
	                (VariableNode "$x")
	                (EvaluationLink
	                    (PredicateNode "smokes")
	                    (ListLink
	                        (VariableNode "$x")
	                    )
	                )
	            )
	        )
	     )
	)
)
