(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		a
		b
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "AndCreationRule<2>")
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
		(AndLink a b (stv 0.160000 0.650000))
	)
)

