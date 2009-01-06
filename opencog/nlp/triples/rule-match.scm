scm
;
; rule-match.scm
;
; Try to match up relex predicates
;
; 
; --------------------------------------------------------------------
;
; Generalized predicate
; The structure of the generalized predicate is:
; 
;    EvaluationLink
;        var-a
;        ListLink
;            var-b
;            var-c
;
;
(define x
	(ImplicationLink 
		(AndLink
			(EvaluationLink
				(PredicateNode "_obj")
				(ListLink
					(ConceptNode "make")
					(VariableNode "$var0")
				)
			)
			(EvaluationLink
				(PredicateNode "from")
				(ListLink
					(ConceptNode "make")
					(VariableNode "$var1")
				)
			)
		)
		(EvaluationLink
			(PredicateNode "make_from")
			(ListLink
				(VariableNode "$var0")
				(VariableNode "$var1")
			)
		)
	)
)

(define o
	(EvaluationLink
		(PredicateNode "_obj")
		(ListLink
			(ConceptNode "make")
			(ConceptNode "pottery")
		)
	)
)
(define f
	(EvaluationLink
		(PredicateNode "from")
		(ListLink
			(ConceptNode "make")
			(ConceptNode "clay")
		)
	)
)

(cog-ad-hoc "do-implication" x)
