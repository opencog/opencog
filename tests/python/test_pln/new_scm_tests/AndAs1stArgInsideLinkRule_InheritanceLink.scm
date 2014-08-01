(define dogs (ConceptNode "dogs" (stv 0.01 1)))
(define animals (ConceptNode "animals" (stv 0.01 1)))
(define bulldogs (ConceptNode "bulldogs" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
	    dogs
	    animals
	    bulldogs
	    (AndLink (stv 0.5 1)
	        (InheritanceLink (stv 0.5 1) dogs animals)
	        (InheritanceLink (stv 0.5 1) bulldogs animals)
	    )
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
	     (InheritanceLink (AndLink dogs bulldogs) animals)
    )
)
