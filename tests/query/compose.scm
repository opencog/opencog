;
; data for ComposeUTest
;
; First, we test double-definition. Here is a very bogus define.
(define (defun-a)
	(DefineLink
		(ConceptNode "some-name")
		(SatisfactionLink
			(VariableNode "$some-var")
			(AndLink
				(ConceptNode "random-atom")
				(VariableNode "$some-var")
			)
		)
	)
)

; a different definition, but with the same name as above.
(define (defun-b)
	(DefineLink
		(ConceptNode "some-name")
		(SatisfactionLink
			(VariableNode "$other-var")
			(ListLink
				(VariableNode "$other-var")
				(ConceptNode "other-atom")
			)
		)
	)
)

; A compose link that should compose with defun-a
(define composer
	(ComposeLink
		(ConceptNode "some-name")
		(ListLink
			(ConceptNode "bogosity")
		)
	)
)

; The expected result of composing defun-a with compser
(define bogo-a
	(AndLink
		(ConceptNode "random-atom")
		(ConceptNode "bogosity")
	)
)

