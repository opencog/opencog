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
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(OrLink P Q (stv 0.1 0.1))
		(SubsetLink (stv 0.100000 0.100000)
			(NotLink P)
			Q
		)
	)
)



