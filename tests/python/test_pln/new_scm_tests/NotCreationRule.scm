(define a (ConceptNode "A" (stv 0.8 0.8)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		a
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "NotCreationRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		a
		(NotLink (stv 0.200000 0.800000) a)
	)
)

