;
; quote-throw.scm
;
; Test invalid use of quote.
;

(EvaluationLink
	(PredicateNode "all-var")
	(ListLink
		(VariableNode "$var-a")
		(VariableNode "$var-b")
	)
)

(define bindy
	(BindLink
		(VariableNode "$var-a")
		(ImplicationLink
			(EvaluationLink
				(VariableNode "$var-a")
				; quote cannot have two things under it; this should cause
				; an exception to be thrown from the pattern matcher.
				(QuoteLink
					(VariableNode "$var-a")
					(VariableNode "$var-b")
				)
			)
			(VariableNode "$var-a")
		)
	)
)

