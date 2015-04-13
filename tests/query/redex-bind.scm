;
; data for RedexBindUTest
;
;
(use-modules (opencog))
(use-modules (opencog query))

;; The data that we expect to be able to match
(EvaluationLink
	(PredicateNode "some-pred")
	(ListLink
		(ConceptNode "thing-one")
		(ConceptNode "thing-two")
	)
)

(define expected-answer
	(SetLink
		(InheritanceLink
			(ConceptNode "thing-two")
			(ConceptNode "thing-one")
		)
	)
)

;; the named pattern
(define defined-pattern
	(DefineLink
		(ConceptNode "two-argument-pattern")
		(SatisfactionLink
			;; First, some simple variable definitions
			(VariableList
				(VariableNode "$first-arg")
				(VariableNode "$second-arg")
			)
			;; This is what should get substituted.
			(EvaluationLink
				(PredicateNode "some-pred")
				(ListLink
					(VariableNode "$second-arg")
					(VariableNode "$first-arg")
				)
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
		(ImplicationLink
			(BetaRedex
				(ConceptNode "two-argument-pattern")
				(ListLink
					(VariableNode "$var-one")
					(VariableNode "$var-two")
				)
			)
			(InheritanceLink
				(VariableNode "$var-one")
				(VariableNode "$var-two")
			)
		)
	)
)
