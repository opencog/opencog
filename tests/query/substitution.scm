;
; Very very basic variable substitution unit test
;
(define varlist
	(VariableList
		(Variable "$a")
		(Variable "$b")
	)
)

(define template
	(EvaluationList
		(PrediceNode "something")
		(ListLink
			(VariableNode "$b")		; note the reversed order
			(VariableNode "$a")
		)
	)
)

(define arglist
	(ListLink
		(ConceptNode "one")
		(NumberNode 2.0000)
	)
)

(define anser
	(EvaluationList
		(PrediceNode "something")
		(ListLink
			(NumberNode 2.0000)
			(ConceptNode "one")
		)
	)
)
