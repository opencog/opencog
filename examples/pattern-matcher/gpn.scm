
(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Linas")
		(ConceptNode "human")
	)
)

(define (say-hello atom)
	(display "hello!")
	(newline)
	(stv 0.5 0.5)
)

(define find-humans
	(BindLink
		(VariableNode "$person")
		(ImplicationLink
			(EvaluationLink
		   	(PredicateNode "is-a")
   			(ListLink
					(VariableNode "$person")
      			(ConceptNode "human")
   			)
			)
			(EvaluationLink
		   	(GroundedPredicateNode "scm: say-hello")
   			(ListLink
					(VariableNode "$person")
   			)
			)
		)
	)
)

; (cog-bind find-humans)

(cog-evaluate!  (EvaluationLink
      (GroundedPredicateNode "scm: say-hello")
      (ListLink
         (ConceptNode "Linas")
      )
   )
)

