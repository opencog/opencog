;
; buggy-link.scm
;
; Test a buggy binding issue.
;

(EvaluationLink
	(PredicateNode "eat")
	(ListLink
		(ConceptNode "Bob")
		(ConceptNode "Doughnut")
	)
)

(EvaluationLink
	(PredicateNode "eat")
	(ListLink
		(ConceptNode "Lily")
		(ConceptNode "Cabbage")
	)
)

; This one causes problems:  the old code used to accept this as
; a valid grounding for one of the clauses, resulting in $var_1
; being undefined, which subsequently leads to a crash. The new
; code explicitly rejects this match.
(EvaluationLink
	(PredicateNode "eat")
	(ListLink
		(VariableNode "$var_1")
		(ConceptNode "Cabbage")
	)
)

(EvaluationLink
	(PredicateNode "like")
	(ListLink
		(ConceptNode "Lily")
		(ConceptNode "apple")
	)
)

(define bindy
	(BindLink
		(ListLink
			(VariableNode "$var_1")
			(VariableNode "$var_2")
		)
		(ImplicationLink
			(AndLink
				(EvaluationLink
					(VariableNode "$var_2")
					(ListLink
						(ConceptNode "Bob")
						(ConceptNode "Doughnut")
					)
				)

				(EvaluationLink
					(VariableNode "$var_2")
					(ListLink
						(VariableNode "$var_1")
						(ConceptNode "Cabbage")
					)
				)

				(EvaluationLink
					(PredicateNode "like")
					(ListLink
						(VariableNode "$var_1")
						(ConceptNode "apple")
					)
				)
			)
			(ListLink
				(VariableNode "$var_1")
				(VariableNode "$var_2")
			)
		)
	)
)

