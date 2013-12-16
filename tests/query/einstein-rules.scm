;;
;; einstein-rules.scm
;;
;; Rules for deducing the answer to the Einstein Puzzle
;; 

(define (puzzle-impl)
	(BindLink
		;; variable decls
		(ListLink
			(VariableNode "$person")
			(VariableNode "$house")
		)
		(ImplicationLink
			;; body
			(EvaluationLink
				(PredicateNode "is_nationality")
				(ListLink
					(ConceptNode "British")
					(VariableNode "$person")
				)
			)
			(EvaluationLink
				(PredicateNode "livesIn")
				(ListLink
					(ConceptNode "British")
					(VariableNode "$person")
				)
			)
			;; implicand -- result
			(ListLink
				(VariableNode "$person")
				(VariableNode "$house")
			)
		)
	)
)

