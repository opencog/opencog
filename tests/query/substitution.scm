;
; Very very basic variable substitution unit test
;
(define varlist
	(VariableList
		(VariableNode "$a")
		(VariableNode "$b")
	)
)

(define template
	(EvaluationLink
		(PredicateNode "something")
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

(define answer
	(EvaluationLink
		(PredicateNode "something")
		(ListLink
			(NumberNode 2.0000)
			(ConceptNode "one")
		)
	)
)
