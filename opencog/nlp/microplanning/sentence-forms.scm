; wrapping into a function so it can be dynamically recreated
(define (microplanning-init)
	(InheritanceLink
		(ConceptNode "DeclarativeUtterance")
		(OrLink
			(EvaluationLink
				(PredicateNode "MicroplanningVerbMarker")
				(Node "MicroplanningWildcardMarker")
			)
		)
	)
	
	(InheritanceLink
		(ConceptNode "InterrogativeUtterance")
		(OrLink
			(EvaluationLink
				(PredicateNode "MicroplanningVerbMarker")
				(Node "MicroplanningWildcardMarker")
			)
			(EvaluationLink
				(VariableNode "MicroplanningAnyNameMarker")
				(ListLink
					(ConceptNode "MicroplanningAnyNameMarker")
				)
			)
		)
	)
)

