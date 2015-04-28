(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.4 0.6)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		a
		b
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "OrCreationRule<2>")
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
		b
		(OrLink (stv 1.000000 0.600000) a b)
	)
)

