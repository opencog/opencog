;
; quote-var.scm
;
; Test simple quoted variables.  The pattern will search for a 
; quoted variable.
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

; We expect this one to be found by the bindy pattern below.
(EvaluationLink
	(PredicateNode "similar")
	(ListLink
		(VariableNode "$var-a")
		(ConceptNode "banana")
	)
)

; The bindy pattern must not find this one.
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
				(PredicateNode "similar")
				(ListLink
					(QuoteLink (VariableNode "$var-a"))
					(VariableNode "$var-a")
				)
			)
			(VariableNode "$var-a")
		)
	)
)

(define bother
	(BindLink
		(VariableNode "$other")
		(ImplicationLink
			(EvaluationLink
				(PredicateNode "similar")
				(ListLink
					(QuoteLink (VariableNode "$var-a"))
					(VariableNode "$other")
				)
			)
			(VariableNode "$other")
		)
	)
)

(define bunbound
	(BindLink
		(VariableNode "$other")
		(ImplicationLink
			(EvaluationLink
				(PredicateNode "similar")
				(ListLink
					(VariableNode "$var-a")
					(VariableNode "$other")
				)
			)
			(VariableNode "$other")
		)
	)
)
