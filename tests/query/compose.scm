;
; data for ComposeUTest
;
;
;; The data that we expect to find
(define expected-answer
	(EvaluationLink
		(PredicateNode "some-pred")
		(ListLink
			(ConceptNode "thing-one")
			(ConceptNode "thing-two")
		)
	)
)

;; the named pattern
(DefineLink
	(ConceptNode "two-argument-pattern`")
	(SatsisfactionLink
		;; First, some simple variable definitions
		(VariableList
			(VariableNode "$first-arg")
			(VariableNode "$second-arg")
		)
		;; This is waht should get substituted.
		(EvaluationLink
			(PredicateNode "some-pred")
			(ListLink
				(VariableNode "$second-arg")
				(VariableNode "$first-arg")
			)
		)
	)
)

;; The Bindlink  we will use to find the data
(define (simple-call)
	(BindLink
		(VariableList
			(VariableNode "$var-one")
			(VariableNode "$var-two")
		)
		(ComposeLink
			(ConceptNode "two-argument-pattern`")
			(ListLink
				(VariableNode "$var-one")
				(VariableNode "$var-two")
			)
		)
	)
)
