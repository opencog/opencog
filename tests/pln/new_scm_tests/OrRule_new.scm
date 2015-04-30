(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))
(define or_ab (OrLink a b))

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
		(OrLink a b (stv 1.000000 0.650000))
	)
)
