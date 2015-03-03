;
; sequence.scm
;
; Test data for the GroundedPredicateNode, which is a virtual node.
; A sequence of these is evaluated, verifying that sequential evaluation
; works.
;

(define green-light  (ConceptNode "green light"))
(define red-light  (ConceptNode "red light"))

(define (stop-go atom)
	(cond
		((eq? atom green-light) (stv 1 1))
		((eq? atom red-light) (stv 0 1))
		(else (throw 'not-a-stoplight))
	)
)

(define (traffic)
	(BindLink
		(ListLink)
		(ImplicationLink
			(AndLink
				(EvaluationLink
					(GroundedPredicateNode "scm: stop-go")
					(ListLink green-light)
				)

				(EvaluationLink
					(GroundedPredicateNode "scm: stop-go")
					(ListLink green-light)
				)

				(EvaluationLink
					(GroundedPredicateNode "scm: stop-go")
					(ListLink red-light)
				)

				(EvaluationLink
					(GroundedPredicateNode "scm: stop-go")
					(ListLink
						(ConceptNode "traffic ticket")
					)
				)
			)

			(ListLink)
		)
	)
)
