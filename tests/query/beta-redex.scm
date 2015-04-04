;
; data for BetaRedexUTest
;
; First, we test double-definition. Here is a very bogus define.
(define (defun-a)
	(DefineLink
		(ConceptNode "some-name")
		(SatisfactionLink
			(VariableNode "$some-var")
			(AssociativeLink
				(ConceptNode "random-atom")
				(VariableNode "$some-var")
			)
		)
	)
)

; A different definition, but with the same name as above.
(define (defun-b)
	(DefineLink
		(ConceptNode "some-name")
		(SatisfactionLink
			(VariableNode "$other-var")
			(InheritanceLink
				(VariableNode "$other-var")
				(ConceptNode "other-atom")
			)
		)
	)
)

; A redex link that should compose with defun-a
(define composer
	(BetaRedex
		(ConceptNode "some-name")
		(ListLink
			(ConceptNode "bogosity")
		)
	)
)

; The expected result of beta-reducing defun-a with composer
(define bogo-a
	(AssociativeLink
		(ConceptNode "random-atom")
		(ConceptNode "bogosity")
	)
)

; A different definition, but with a quote link
(define (defun-quote)
	(DefineLink
		(ConceptNode "quoter")
		(SatisfactionLink
			(VariableNode "$other-var")
			(AttractionLink
				(VariableNode "$other-var")
				(ConceptNode "other-atom")
				(QuoteLink
					(VariableNode "$other-var")
				)
			)
		)
	)
)

; A redex link that should compose with defun-quote
(define compose-quote
	(BetaRedex
		(ConceptNode "quoter")
		(ListLink
			(ConceptNode "yeah, right")
		)
	)
)

; The expected result of composing defun-a with composer
(define yeah-quote
	(AttractionLink
		(ConceptNode "yeah, right")
		(ConceptNode "other-atom")
		(QuoteLink
			(VariableNode "$other-var")
		)
	)
)
