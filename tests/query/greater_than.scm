;
; greater_than.scm
;
; Test data for the GroundedPredicateNode, which is a virtual node.
; This declares the net worth of four individuals.  It also declares
; four BindLinks, which find everyone who is richer.  The greater-than
; comparison of richness is performed via the virtual link evaluation.
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

(define (richer-than-person-x-cmp person-x comp-fun)
	(BindLink
		(VariableList
			(VariableNode "$who")
			(TypedVariableLink
				(VariableNode "$less-wealth")
				(TypeNode "NumberNode")
			)
			(TypedVariableLink
				(VariableNode "$more-wealth")
				(TypeNode "NumberNode")
			)
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
					comp-fun
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

;; This variant uses the built-in c++ greater-than code.
(define (richer-than-person-x person-x)
	(richer-than-person-x-cmp person-x (GroundedPredicateNode "c++:greater")))

(define (richer-than-gates)
	(richer-than-person-x (ConceptNode "Bill Gates")))

(define (richer-than-obama)
	(richer-than-person-x (ConceptNode "Obama")))

(define (richer-than-george)
	(richer-than-person-x (ConceptNode "George P. from Waxahachie")))

(define (richer-than-susan)
	(richer-than-person-x (ConceptNode "Susan M. from Peoria")))


;; This variant uses a hand-rolled scm compare function
(define (richer a b)
	(if (> (string->number (cog-name a)) (string->number (cog-name b)))
		(stv 1 1)  ;; true
		(stv 0 1)  ;; false
	)
)

;; Use the "richer" function defined immediately above.
(define (scm-than-person-x person-x)
	(richer-than-person-x-cmp person-x (GroundedPredicateNode "scm:richer")))

(define (scm-than-gates)
	(scm-than-person-x (ConceptNode "Bill Gates")))

(define (scm-than-obama)
	(scm-than-person-x (ConceptNode "Obama")))

(define (scm-than-george)
	(scm-than-person-x (ConceptNode "George P. from Waxahachie")))

(define (scm-than-susan)
	(scm-than-person-x (ConceptNode "Susan M. from Peoria")))


