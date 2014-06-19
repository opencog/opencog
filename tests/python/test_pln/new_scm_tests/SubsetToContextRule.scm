; "Dogs are animals."

(define dogs (ConceptNode "dogs" (stv 0.01 1)))
(define animals (ConceptNode "animals" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
		 (SubsetLink (stv 0.5 1)
            dogs
            animals
         )
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "SubsetToContextRule")
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
	    (SubsetLink (stv 0.5 1)
            dogs
            animals
         )
	    (ContextLink (stv 0.5 1)
	        dogs
            animals
        )
    )
)
