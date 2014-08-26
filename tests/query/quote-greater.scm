;
; quote-greater.scm
;
; Test data for the QuoteLink -- should ignore the GroundedPredicateNode
; This is very similar to the greater-than unit test, except that the
; GPN is explicitly disabled with a quote.
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

;; this is thoroughly bogus, but that's intentional: the quotelink
;; is being tested, and so we put some non-virtual data into the
;; the atomspace.
(EvaluationLink
	(GroundedPredicateNode "c++:greater")
	(ListLink
		(NumberNode "1000")
		(NumberNode "310")
	)
)
(EvaluationLink
	(GroundedPredicateNode "c++:greater")
	(ListLink
		(NumberNode "310")
		(NumberNode "200")
	)
)
(EvaluationLink
	(GroundedPredicateNode "c++:greater")
	(ListLink
		(NumberNode "1000")
		(NumberNode "200")
	)
)

;; The actual BindLink that will be tested.
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

				(EvaluationLink
					(QuoteLink
						(GroundedPredicateNode "c++:greater")
					)
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


