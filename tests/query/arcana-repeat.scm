;
; Unit testing for a strange repeated-clause
;
;;; Populate the atomspace with two things
(ImplicationLink
	(ListLink
		(EvaluationLink
			(PredicateNode "this way")
			(ListLink
				(ConceptNode "this one")
				(ConceptNode "thing two")
			)
		)
	)
	(MemberLink
		(EvaluationLink
			(PredicateNode "this way")
			(ListLink
				(ConceptNode "this one")
				(ConceptNode "thing two")
			)
		)
	)
)

(ImplicationLink
	(ListLink
		(EvaluationLink
			(PredicateNode "this way")
			(ListLink
				(ConceptNode "this one")
				(ConceptNode "thing two")
			)
		)
	)
	(ListLink
		(EvaluationLink
			(PredicateNode "this way")
			(ListLink
				(ConceptNode "this one")
				(ConceptNode "thing two")
			)
		)
	)
)

;;; Note that the evaluationLink is repeated twice, inside of
;;; a ListLink, each time.
(define (repeat-same)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(ImplicationLink
				(ListLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
				)
				(ListLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
;;; Note that the evaluationLink is repeated twice, inside of
;;; two different links
(define (repeat-different)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(ImplicationLink
				(ListLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
				)
				(MemberLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
