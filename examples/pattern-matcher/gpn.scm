
(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

(define (rand-ok atom)
	(if (< 0 (random 2))
		(begin
			(simple-format #t "Picked ~A\n" (cog-name atom))
			(stv 1 1) ; return true
		)
		(begin
			(simple-format #t "Did not pick ~A\n" (cog-name atom))
			(stv 0 1) ; return true
		)
	)
)

(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Aristotle")
		(ConceptNode "human")
	)
)

(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Pierce")
		(ConceptNode "human")
	)
)

(define find-humans
	(BindLink
		(VariableNode "$person")
		(ImplicationLink
			(AndLink
				(EvaluationLink
					(PredicateNode "is-a")
					(ListLink
						(VariableNode "$person")
						(ConceptNode "human")
					)
				)
				(EvaluationLink
					(GroundedPredicateNode "scm: rand-ok")
					(ListLink
						(VariableNode "$person")
					)
				)
			)
			(VariableNode "$person")
		)
	)
)

; (cog-bind find-humans)


