; wrapping into a function so it can be dynamically recreated
(define (microplanning-init)
	(InheritanceLink
		(ConceptNode "declarative")
		(OrLink
			(EvaluationLink
				(PredicateNode "verb")
				(AnyNode "_")
			)
		)
	)
	
	(InheritanceLink
		(ConceptNode "interrogative")
		(OrLink
			(EvaluationLink
				(PredicateNode "verb")
				(AnyNode "_")
			)
			(EvaluationLink
				(VariableNode "_")
				(ListLink
					(ConceptNode "_")
				)
			)
		)
	)
	
)
		
