;
; Basic test of unification
;
; Create some misc atoms.

(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))

(add-to-load-path ".")
(add-to-load-path "./build")
(add-to-load-path "../build")

(load-from-path "utilities.scm")
(load-from-path "opencog/nlp/types/nlp_types.scm")

(define (stv mean conf) (cog-new-stv mean conf))

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
		(WordNode "capital@ggg")
		(WordNode "Germany")
	)
)
(EvaluationLink
	(PredicateNode "of")
	(ListLink
		(WordNode "capital@fff")
		(WordNode "France")
	)
)
(EvaluationLink
	(PredicateNode "_subj")
	(ListLink
		(WordNode "be@fff")
		(WordNode "capital@fff")
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
		(WordNode "capital@ggg")
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
		(VariableList
			(VariableNode "$be-word")
			(VariableNode "$capital")
			(VariableNode "$country")
			(VariableNode "$city")
		)
		impl
	)
)

