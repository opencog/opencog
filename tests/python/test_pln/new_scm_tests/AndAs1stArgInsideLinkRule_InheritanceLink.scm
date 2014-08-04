(define dogs (ConceptNode "dogs" (stv 0.01 1)))
(define bulldogs (ConceptNode "bulldogs" (stv 0.01 1)))
(define animals (ConceptNode "animals" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
	    dogs
	    bulldogs
	    animals
	    (InheritanceLink dogs animals (stv 0.5 1))
	    (InheritanceLink bulldogs animals (stv 0.5 1))
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "AndAs1stArgInsideLinkRule<InheritanceLink>")
	)
)

(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs")
	(ListLink
	     dogs
	     animals
	     bulldogs
	     (InheritanceLink dogs animals (stv 0.5 1))
	     (InheritanceLink bulldogs animals (stv 0.5 1))
	     (InheritanceLink (stv .5 1.0)
	        (AndLink dogs bulldogs (stv .005 1.0))
	        animals
	     )
    )
)
