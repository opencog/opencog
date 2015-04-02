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

; a more complex template
(define free-template
	(EvaluationLink
		(PredicateNode "something")
		(ListLink
			(VariableNode "$b")		; note the reversed order
			(VariableNode "$c")
			(VariableNode "$a")
			(ListLink
				(VariableNode "$d")
			)
			(VariableNode "$a")
			(VariableNode "$a")
		)
	)
)

;; answer for above
(define free-answer
	(EvaluationLink
		(PredicateNode "something")
		(ListLink
			(NumberNode 2.0000)
			(VariableNode "$c")
			(ConceptNode "one")
			(ListLink
				(VariableNode "$d")
			)
			(ConceptNode "one")
			(ConceptNode "one")
		)
	)
)

(define typed-varlist
	(VariableList
		(TypedVariableLink
			(VariableNode "$a")
			(TypeNode "ConceptNode")
		)
		(TypedVariableLink
			(VariableNode "$b")
			(TypeChoice
				(TypeNode "NumberNode")
				(TypeNode "AnchorNode")
			)
		)
	)
)

;; This must not match the typed varlist above
(define bad-arglist
	(ListLink
		(NumberNode 1.0000)
		(ConceptNode "two")
	)
)

;; malformed typelist
;; This should throw exception upon load
(define bad-varlist
	(VariableList
		(TypedVariableLink
			(VariableNode "$a")
			(TypeNode "ConceptNodeNode")  ;; intentionally mis-spelled
		)
		(TypedVariableLink
			(VariableNode "$b")
			(TypeChoice
				(TypeNode "NumberNode")
				(TypeNode "AnchorNode")
			)
		)
	)
)
