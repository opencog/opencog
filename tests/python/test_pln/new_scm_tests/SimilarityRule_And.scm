(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink
		(AndLink (stv 0.3 0.8) a b)
		(OrLink (stv 0.5 0.1) a b)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "SimilarityRule")
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
		(AndLink (stv 0.3 0.8) a b)
		(SimilarityLink (stv 0.1 0.1) a b)
	)
)

