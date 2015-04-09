;
; Unit testing for OrLinks in the pattern matcher.
;
;;; Populate the atomspace with three small trees.
(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "this way")
		(ListLink
			(ConceptNode "this one")
			(ConceptNode "thing two")
		)
	)
)

(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "that way")
		(ListLink
			(ConceptNode "thing one")
			(ConceptNode "that too")
		)
	)
)

(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "third way")
		(ListLink
			(ConceptNode "thing one")
			(ConceptNode "thing two")
		)
	)
)

;;; One clause, with an OrLink nested in it.
(define (embed)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(MemberLink
				(ConceptNode "ways and means")
				(OrLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
					(EvaluationLink
						(PredicateNode "that way")
						(ListLink
							(ConceptNode "thing one")
							(VariableNode "$x")
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
