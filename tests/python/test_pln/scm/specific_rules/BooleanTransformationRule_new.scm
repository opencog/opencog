; P OR Q = Not(P) => Q

(define P (ConceptNode "P"))
(define Q (ConceptNode "Q"))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(OrLink P Q (stv 0.1 0.1))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "BooleanTransformationRule<OrLink,SubsetLink>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(OrLink P Q (stv 0.1 0.1))
		(SubsetLink
			(NotLink P)
			Q
		)
	)
)



