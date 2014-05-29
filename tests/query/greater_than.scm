;
; Test data for the GreaterThanLink
;
;
(EvaluationLink
	(PredicateNode "net-worth")
	(ListLink
		(ConceptNode "Bill Gates")
		(NumberNode "500000")
	)
)

(EvaluationLink
	(PredicateNode "net-worth")
	(ListLink
		(ConceptNode "Obama")
		(NumberNode "1000")
	)
)

(EvaluationLink
	(PredicateNode "net-worth")
	(ListLink
		(ConceptNode "Susan M. from Peoria")
		(NumberNode "200")
	)
)

(EvaluationLink
	(PredicateNode "net-worth")
	(ListLink
		(ConceptNode "George P. from Waxahachie")
		(NumberNode "310")
	)
)

(define (richer-than-person-x person-x)
	(BindLink
		(ListLink
			(VariableNode "$who")
			(VariableNode "$less-wealth")
			(VariableNode "$more-wealth")
		)
		(ImplicationLink
			(AndLink
				(EvaluationLink
					(PredicateNode "net-worth")
					(ListLink
						person-x
						(VariableNode "$less-wealth")
					)
				)

				(EvaluationLink
					(PredicateNode "net-worth")
					(ListLink
						(VariableNode "$who")
						(VariableNode "$more-wealth")
					)
				)

				(GreaterThanLink
					(GroundedSchemaNode "c++:greater")
					(ListLink
						(VariableNode "$more-wealth")
						(VariableNode "$less-wealth")
					)
				)
			)

			;; output result: just the concept node of who it is.
			(VariableNode "$who")
		)
	)
)

(define (richer-than-gates)
	(richer-than-person-x (ConceptNode "Bill Gates")))

(define (richer-than-obama)
	(richer-than-person-x (ConceptNode "Obama")))

(define (richer-than-george)
	(richer-than-person-x (ConceptNode "George P. from Waxahachie")))

(define (richer-than-susan)
	(richer-than-person-x (ConceptNode "Susan M. from Peoria")))


