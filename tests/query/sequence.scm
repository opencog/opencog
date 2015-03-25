;
; sequence.scm
;
; Test data for the GroundedPredicateNode, which is a virtual node.
; A sequence of these is evaluated, verifying that sequential evaluation
; works.
;

(define green-light  (ConceptNode "green light"))
(define red-light  (ConceptNode "red light"))

(define num-green 0)
(define num-red 0)

; Return SimpleTV of true if green light, false if red light, and
; throw an exception if neither.  Increment counters so that we
; can verify that this was invoked.
(define (stop-go atom)
	(cond
		((equal? atom green-light) (begin (set! num-green (+ 1 num-green)) (stv 1 1)))
		((equal? atom red-light) (begin (set! num-red (+ 1 num-red)) (stv 0 1)))
		(else (throw 'not-a-stoplight "stop-go" "you're busted"))
	)
)

; Should throw an exception in all cases. Shouldn't do donuts on
; corn fields.
(define (off-road)
	(BindLink
		(SignatureLink)  ; no variables
		(ImplicationLink
			(SequentialAndLink
				(EvaluationLink
					(GroundedPredicateNode "scm: stop-go")
					(ListLink
						(ConceptNode "corn field")
					)
				)
			)
			(ListLink)  ; no returned value
		)
	)
)

;; Should see two green lights, and one red light, after which
;; the matching should stop.  There should be no exceptions or
;; errors when evaluating this.
(define (traffic-lights)
	(BindLink
		(SignatureLink)  ; no variables
		(ImplicationLink
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
			(ListLink) ; no returned value
		)
	)
)
