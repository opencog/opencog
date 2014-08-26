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
		(ConceptNode "bananna")
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
		(ConceptNode "bananna")
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
		(VariableNode "$wrong-var-a")
		(ConceptNode "apple")
	)
)

(define bindy
	(BindLink
		(VariableNode "$var-a")
		(ImplicationLink
			(EvaluationLink
				(PredicateNode "$var-a")
				(QuoteLink (ListLink
					(VariableNode "$var-a")
					(VariableNode "$var-b")
				))
			)
			(VariableNode "$var-a")
		)
	)
)

