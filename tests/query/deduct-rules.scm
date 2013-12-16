;;
;; deduct-rules.scm
;;
;; Deduction rules for Einstein puzzle
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; "Is the same person" deduction rule.
;; If person A and person B both share the same predicate and property,
;; then they must be the same person.
(define (is-same-rule)
	(BindLink
		;; variable declrations
		(ListLink
         (TypedVariableLink
            (VariableNode "$predicate")
            (VariableTypeNode "PredicateNode")
         )
         (TypedVariableLink
            (VariableNode "$person_a")
            (VariableTypeNode "AvatarNode")
         )
         (TypedVariableLink
            (VariableNode "$person_b")
            (VariableTypeNode "AvatarNode")
         )
         (TypedVariableLink
            (VariableNode "$property")
            (VariableTypeNode "ConceptNode")
         )
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(EvaluationLink
					(VariableNode "$predicate")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$property")
					)
				)
				(EvaluationLink
					(VariableNode "$predicate")
					(ListLink
						(VariableNode "$person_b")
						(VariableNode "$property")
					)
				)
			)
			;; implicand -- then the following is true too
			(EvaluationLink
				(PredicateNode "IsSamePerson")
				(ListLink
					(VariableNode "$person_a")
					(VariableNode "$person_b")
				)
			)
		)
	)
)



