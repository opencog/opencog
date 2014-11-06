; wrapping into a function so it can be dynamically recreated
(define (microplanning-init)
	(InheritanceLink
		(ConceptNode "declarative")
		(OrLink
			; SV
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
				)
			)
			; SVO
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
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
	
	(InheritanceLink
		(ConceptNode "interrogative")
		(OrLink
			; SV
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
				)
			)
			(EvaluationLink
				(VariableNode "_")
				(ListLink
					(ConceptNode "_")
				)
			)
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(VariableNode "_")
				)
			)
			; SVO
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(ConceptNode "_")
				)
			)
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(VariableNode "_")
				)
			)
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(VariableNode "_")
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
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(VariableNode "_")
					(ConceptNode "_")
					(ConceptNode "_")
				)
			)
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(VariableNode "_")
					(ConceptNode "_")
				)
			)
			(EvaluationLink
				(PredicateNode "verb")
				(ListLink
					(ConceptNode "_")
					(ConceptNode "_")
					(VariableNode "_")
				)
			)
		)
	)
	
)
		
