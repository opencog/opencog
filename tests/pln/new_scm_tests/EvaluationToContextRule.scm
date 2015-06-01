; "In the context of the earth, the sky is blue.
;  In the context of the moon, the sky is not blue."

(define isBlue (PredicateNode "isBlue" (stv 0.01 1)))
(define sky (ConceptNode "sky" (stv 0.01 1)))
(define earth (ConceptNode "earth" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
		 (EvaluationLink (stv 0.5 1)
            isBlue
            (ListLink
                (AndLink sky earth)
            )
        )
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "EvaluationToContextRule")
	)
)

(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs")
	(ListLink
	     isBlue
	     sky
	     earth
	     (AndLink sky earth)
	     (ContextLink (stv 0.5 1)
	        earth
            (EvaluationLink
                isBlue
                (ListLink sky)
            )
        )
    )
)
