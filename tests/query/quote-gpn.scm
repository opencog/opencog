;
; quote-gpn.scm
;
; Test QuoteLink -- check for appropriate search scoping.
;

; some data
(EvaluationLink
	(GroundedPredicateNode "scm:do_stuff")
	(ListLink
		(ConceptNode "thing-a")
		(ConceptNode "thing-b")
	)
)

; The pattern below can confuse the search start, becuase
; the first constant link in the clause is the quote ... 
; and that's won't provide the desired start ...
(define bindy
	(BindLink
		(VariableNode "$stuff")
		(ImplicationLink
			(EvaluationLink
				(QuoteLink (GroundedPredicateNode "scm:do_stuff"))
				(VariableNode "$stuff")
			)
			(VariableNode "$stuff")
		)
	)
)
