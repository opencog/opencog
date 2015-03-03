;
; sequence.scm
;
; Test data for the GroundedPredicateNode, which is a virtual node.
; A sequence of these is evaluated, verifying that sequential evaluation
; works.
;

(define green-light  (ConceptNode "green light"))
(define red-light  (ConceptNode "red light"))

; Return SimpleTV of true if green light, false if red light, and
; throw an exception if neither.
(define (stop-go atom)
	(cond
		((eq? atom green-light) (begin (display "go\n") (stv 1 1)))
		((eq? atom red-light) (begin (display "stop\n") (stv 0 1)))
		(else (begin (display "busted\n") (throw 'not-a-stoplight)))
	)
)

; Should throw an exception in all cases. Shouldn't do donuts on
; corn fields.
(define (off-road)
	(BindLink
		(ListLink)
		(ImplicationLink
			(AndLink
				(SequentialAndLink
					(EvaluationLink
						(GroundedPredicateNode "scm: stop-go")
						(ListLink
							(ConceptNode "corn field")
						)
					)
				)
			)

			(ListLink)
		)
	)
)

(define (traffic-lights)
	(BindLink
		(ListLink)
		(ImplicationLink
			(AndLink
				(SequentialAndLink
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
			)

			(ListLink)
		)
	)
)
