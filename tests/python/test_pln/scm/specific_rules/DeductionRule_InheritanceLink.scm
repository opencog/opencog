; The formula for deduction uses at least part of every Node's TV.
(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(ConceptNode "Human" (stv 0.01 1))
		(ConceptNode "Socrates" (stv 0.01 1))
		(ConceptNode "Mortal" (stv 0.01 1))
		(InheritanceLink (ConceptNode "Human") (ConceptNode "Mortal") (stv 0.5 1))
		(InheritanceLink (ConceptNode "Socrates") (ConceptNode "Human") (stv 0.5 1))
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "DeductionRule<InheritanceLink>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(ConceptNode "Human" (stv 0.01 1))
		(ConceptNode "Socrates" (stv 0.01 1))
		(ConceptNode "Mortal" (stv 0.01 1))
		(InheritanceLink (ConceptNode "Human") (ConceptNode "Mortal") (stv 0.5 1))
		(InheritanceLink (ConceptNode "Socrates") (ConceptNode "Human") (stv 0.5 1))
		(InheritanceLink (ConceptNode "Socrates") (ConceptNode "Mortal") (stv 0.252525 1))
	)
)


