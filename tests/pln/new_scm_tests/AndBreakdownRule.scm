(define insane (ConceptNode "Insane" (stv 0.2 0.8)))
(define airesearcher (ConceptNode "AIResearcher" (stv 0.2 0.8)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		insane
		airesearcher
		(AndLink insane airesearcher (stv 0.01 0.8))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "AndBreakdownRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(ConceptNode "Insane" (stv 0.080000 0.909091))
		(ConceptNode "AIResearcher" (stv 0.2 0.8))
		(AndLink (ConceptNode "Insane") (ConceptNode "AIResearcher") (stv 0.01 0.8))
	)
)

