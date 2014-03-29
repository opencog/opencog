;
; Basic test of unification
;
; Create some misc atoms.

(ListLink
	(ConceptNode "hello" (stv 0.5 0.5))
	(ConceptNode "world")
)

(define wobbly
	(ConceptNode "wobbly" (stv 0.5 0.5))
)

(EvaluationLink
	(PredicateNode "of")
	(ListLink
		(WordNode "capital")
		(WordNode "Germany")
	)
)
(EvaluationLink
	(PredicateNode "of")
	(ListLink
		(WordNode "capital")
		(WordNode "France")
	)
)
(EvaluationLink
	(PredicateNode "_subj")
	(ListLink
		(WordNode "be@fff")
		(WordNode "capital")
	)
)
(EvaluationLink
	(PredicateNode "_obj")
	(ListLink
		(WordNode "be@fff")
		(WordNode "Paris")
	)
)
(EvaluationLink
	(PredicateNode "_subj")
	(ListLink
		(WordNode "be@ggg")
		(WordNode "capital")
	)
)
(EvaluationLink
	(PredicateNode "_obj")
	(ListLink
		(WordNode "be@ggg")
		(WordNode "Berlin")
	)
)

; Given subject, object and preposition, deduce the capital.
;
(define impl
	(ImplicationLink
		(AndLink
			(EvaluationLink
				(PredicateNode "_subj")
				(ListLink
					(VariableNode "$be-word")
					(VariableNode "$capital")
				)
			)
			(EvaluationLink
				(PredicateNode "_obj")
				(ListLink
					(VariableNode "$be-word")
					(VariableNode "$city")
				)
			)
			(EvaluationLink
				(PredicateNode "of")
				(ListLink
					(VariableNode "$capital")
					(VariableNode "$country")
				)
			)
		)
		(EvaluationLink
			(PredicateNode "capital-of")
			(ListLink
				(VariableNode "$country")
				(VariableNode "$city")
			)
		)
	)
)

; The actual BindLink
(define cap-deduce
	(BindLink
		(ListLink
			(VariableNode "$be-word")
			(VariableNode "$capital")
			(VariableNode "$country")
			(VariableNode "$city")
		)
		impl
	)
)

