;
; Unit testing for ChoiceLinks in the pattern matcher.
; The Or-pattern is "disconnected": although it has a variable
; in both disjuncts, that variable plays a completely different
; role in each.  Graphically, its really two disconnected graphs
; (where a connection is defined in terms of shared variables)
;
(use-modules (opencog))
(use-modules (opencog query))

;;; Populate the atomspace with three small trees.
(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "this way")
		(ListLink
			(ConceptNode "this one")
			(ConceptNode "thing two")
		)
	)
)

(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "that way")
		(ListLink
			(ConceptNode "thing one")
			(ConceptNode "that too")
		)
	)
)

(MemberLink
	(ConceptNode "ways and means")
	(EvaluationLink
		(PredicateNode "third way")
		(ListLink
			(ConceptNode "thing one")
			(ConceptNode "thing two")
		)
	)
)

;;; One clause, with an ChoiceLink nested in it. Note that the two
;;; parts are entirely disconnected from each-other.
(define (embed-disco)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(MemberLink
				(ConceptNode "ways and means")
				(ChoiceLink
					(EvaluationLink
						(PredicateNode "this way")
						(ListLink
							(VariableNode "$x")
							(ConceptNode "thing two")
						)
					)
					(EvaluationLink
						(PredicateNode "that way")
						(ListLink
							(ConceptNode "thing one")
							(VariableNode "$x")
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
