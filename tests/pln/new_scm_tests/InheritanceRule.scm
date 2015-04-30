(define sub (ConceptNode "sub" (stv 0.5 0.999)))
(define super (ConceptNode "super" (stv 0.5 0.999)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(SubsetLink (stv 0.6 0.6) sub super)
		(IntensionalInheritanceLink (stv 0.6 0.6) sub super)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "InheritanceRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink
		(SubsetLink (stv 0.6 0.6) sub super)
		(IntensionalInheritanceLink (stv 0.6 0.6) sub super)
		(InheritanceLink (stv 0.6 0.6) sub super)		
	)
)
