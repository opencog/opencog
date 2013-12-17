;;
;; deduct-rules.scm
;;
;; Deduction rules for Einstein puzzle
;;

(define (stv mean conf) (cog-new-stv mean conf))

(define (decl-var type var)
	(TypedVariableLink
		(VariableNode var)
		(VariableTypeNode type)
	)
)

;; "Is the same person" deduction rule.
;; If person A and person B both share the same predicate and property,
;; then they must be the same person.
(define (is-same-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "PredicateNode" "$predicate")
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
			(decl-var "ConceptNode" "$property")
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
				;; Avoid reporting things we already know.
				;; Basically, if we already know that person A and B
				;; are the same person, then lets not deduce it again.
				(NotLink
					(EvaluationLink
						(PredicateNode "IsSamePerson")
						(ListLink
							(VariableNode "$person_a")
							(VariableNode "$person_b")
						)
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


;; transitive deduction rule.
;; If property X holds for person A, and person A is same as person B
;; then property X also holds for person B.
(define (transitive-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "PredicateNode" "$predicate")
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
			(decl-var "ConceptNode" "$property")
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
					(PredicateNode "IsSamePerson")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$person_b")
					)
				)
			)
			;; implicand -- then the following is true too
			(EvaluationLink
				(VariableNode "$predicate")
				(ListLink
					(VariableNode "$person_b")
					(VariableNode "$property")
				)
			)
		)
	)
)



;; neighbor deduction rule.
;; If Address X is left of address Y, then person who lives in X is
;; a neighbor of person who lives in Y
(define (neighbor-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
			(decl-var "ConceptNode" "$house_a")
			(decl-var "ConceptNode" "$house_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(EvaluationLink
					(PredicateNode "Address")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$house_a")
					)
				)
				(EvaluationLink
					(PredicateNode "Address")
					(ListLink
						(VariableNode "$person_b")
						(VariableNode "$house_b")
					)
				)
				(EvaluationLink
					(PredicateNode "Successor")
					(ListLink
						(VariableNode "$house_a")
						(VariableNode "$house_b")
					)
				)
			)
			;; implicand -- then the following is true too
			(EvaluationLink
				(PredicateNode "Neighbor")
				(ListLink
					(VariableNode "$person_a")
					(VariableNode "$person_b")
				)
			)
		)
	)
)



