;;
;; deduct-trivial.scm
;;
;; Trivial example of deduction.
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; The Englishman lives in the red house.
(EvaluationLink (stv 1 1)
	(PredicateNode "Nationality")
	(ListLink
		(AvatarNode "person1")
		(ConceptNode "British")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person1")
		(ConceptNode "red_house")
	)
)

;; The person who lives in the red house keeps fish.
(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person2")
		(ConceptNode "red_house")
	)
)

(EvaluationLink (stv 1 1)
	(PredicateNode "KeepsPet")
	(ListLink
		(AvatarNode "person2")
		(ConceptNode "fish")
	)
)

(define puzzle-impl
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
			(EvaluationLink (stv 1 0.99)
				(PredicateNode "IsSamePerson")
				(ListLink
					(VariableNode "$person_a")
					(VariableNode "$person_b")
				)
			)
		)
	)
)



