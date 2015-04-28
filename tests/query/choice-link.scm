;
; Basic unit testing for ChoiceLinks in the pattern matcher.
;
(use-modules (opencog))
(use-modules (opencog query))

;;; Populate the atomspace with four small trees.
(MemberLink
	(ConceptNode "Tom")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Joe")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Hank")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Tom")
	(ConceptNode "Senator")
)

(MemberLink
	(ConceptNode "Joe")
	(ConceptNode "Representative")
)

;; We should NOT find Hank among the solutions
(MemberLink
	(ConceptNode "Hank")
	(ConceptNode "CEO")
)

;;; Two clauses; they both connected with a common variable.
(define (basic)
	(BindLink
		(ImplicationLink
			(AndLink
				(MemberLink
					(VariableNode "$x")
					(ConceptNode "ways and means")
				)
				(ChoiceLink
					(MemberLink
						(VariableNode "$x")
						(ConceptNode "Senator")
					)
					(MemberLink
						(VariableNode "$x")
						(ConceptNode "Representative")
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
