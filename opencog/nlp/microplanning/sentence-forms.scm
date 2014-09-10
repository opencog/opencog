; wrapping into a function so it can be dynamically recreated
(define (microplanning-init)
	(InheritanceLink
		(ConceptNode "declarative")
		(OrLink
			; SVO
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(ConceptNode "_")
				)
			)
			; SV
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
				)
			)
			; SVIO
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(ConceptNode "_")
					(ConceptNode "_")
				)
			)
		)
	)
)
		
