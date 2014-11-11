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

; The pattern below uses $var-a in a confusing way: both as
; variable, as as something quoted.  It should behave the same
; way that "bother" below does: the quoted and unquoted forms
; are distinct from one-another.
(define bindy
	(BindLink
		(TypedVariableLink
			(VariableNode "$var-a")
			(VariableTypeNode "ConceptNode")
		)
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

; Same as above, but explicitly made non-confusing. This is
; what the above form must actually behave like ...
(define bother
	(BindLink
		(TypedVariableLink
			(VariableNode "$other")
			(VariableTypeNode "ConceptNode")
		)
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
