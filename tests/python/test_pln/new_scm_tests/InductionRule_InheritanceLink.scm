(define dutch (ConceptNode "Dutch" (stv 0.01 1)))
(define alex (ConceptNode "Alex" (stv 0.01 1)))
(define smoker (ConceptNode "Smoker" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(InheritanceLink alex dutch (stv 0.5 1))
		(InheritanceLink alex smoker (stv 0.5 1))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "InductionRule<InheritanceLink>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		dutch
		alex
		smoker
		(InheritanceLink alex smoker (stv 0.5 1))
		(InheritanceLink alex dutch (stv 0.5 1))
		(InheritanceLink dutch smoker (stv 0.252525 1))
	)
)


