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


(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))
(define c (ConceptNode "C" (stv 0.47 0.82)))
(define imp_ab (InheritanceLink a b (stv 0.5 0.75)))
(define imp_bc (InheritanceLink b c (stv 0.6 0.7)))

(define imp_ac (InheritanceLink a c))

(EvaluationLink (PredicateNode "query") (ListLink imp_ac))

