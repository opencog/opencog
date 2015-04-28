;
; quote-nest.scm
;
; Test nested quoted variables.  The pattern will search for
; several quoted variables.
;

(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(ConceptNode "apple")
		(ConceptNode "banana")
	)
)

(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(ConceptNode "orange")
		(ConceptNode "apple")
	)
)

(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(ConceptNode "apple")
		(ConceptNode "grape")
	)
)

(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(VariableNode "$var-a")
		(ConceptNode "banana")
	)
)

(EvaluationLink
	(PredicateNode "all-var")
	(ListLink
		(VariableNode "$var-a")
		(VariableNode "$var-b")
	)
)

(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(VariableNode "$var-a")
		(VariableNode "$wrong-var-b")
	)
)

(define bindy
	(BindLink
		(VariableNode "$var-a")
		(ImplicationLink
			(EvaluationLink
				(VariableNode "$var-a")
				(QuoteLink (ListLink
					(VariableNode "$var-a")
					(VariableNode "$var-b")
				))
			)
			(VariableNode "$var-a")
		)
	)
)

