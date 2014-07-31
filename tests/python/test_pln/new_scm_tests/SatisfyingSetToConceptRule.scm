(EvaluationLink (PredicateNode "inputs")
	(SatisfyingSetLink
	    (VariableNode "$X")
	    (EvaluationLink
	        (PredicateNode "smokes")
	        (ListLink
	            (VariableNode "$X")
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
	            (SatisfyingSetLink
	                (VariableNode "$X")
	                (EvaluationLink
	                    (PredicateNode "smokes")
	                    (ListLink
	                        (VariableNode "$X")
	                    )
	                )
	            )
	        )
	     )
	)
)
