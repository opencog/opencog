(define human (ConceptNode "Human" (stv 0.01 1)))
(define socrates (ConceptNode "Socrates" (stv 0.01 1)))
(define mortal (ConceptNode "Mortal" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(InheritanceLink human mortal (stv 0.5 1))
		(InheritanceLink socrates human (stv 0.5 1))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "EvaluationImplicationRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		human
		socrates
		mortal
		(InheritanceLink human mortal (stv 0.5 1))
		(InheritanceLink socrates human (stv 0.5 1))
		(InheritanceLink socrates mortal (stv 0.252525 1))
	)
)


