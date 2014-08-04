(define dogs (ConceptNode "dogs" (stv 0.01 1)))
(define animals (ConceptNode "animals" (stv 0.01 1)))
(define bulldogs (ConceptNode "bulldogs" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
	    bulldogs
	    dogs
	    animals
	    (InheritanceLink (stv 0.5 1) bulldogs dogs)
	    (InheritanceLink (stv 0.5 1) bulldogs animals)
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "AndAs2ndArgInsideLinkRule<InheritanceLink>")
	)
)

(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs")
	(ListLink
	     bulldogs
	     dogs
	     animals
	     (InheritanceLink (stv 0.5 1) bulldogs dogs)
	     (InheritanceLink (stv 0.5 1) bulldogs animals)
	     (InheritanceLink (stv .0025 1.0)
	        bulldogs
	        (AndLink dogs animals (stv .0001 1.0))
	     )
    )
)
