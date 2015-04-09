;
; Unit testing for OrLinks in the pattern matcher.
;
;;; Populate the atomspace with four small trees.
(MemberLink
	(ConceptNode "Tom")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Joe")
	(ConceptNode "ways and means")
)

;;; the list link serves o purpose other than to "embed"
(ListLink
	(MemberLink
		(ConceptNode "Tom")
		(ConceptNode "Senator")
	)
)

(ListLink
	(MemberLink
		(ConceptNode "Joe")
		(ConceptNode "Representative")
	)
)

;;; Two clauses; they both connected with a common variable.
(define (embed)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(AndLink
				(MemberLink
					(VariableNode "$x")
					(ConceptNode "ways and means")
				)
				(ListLink
					(OrLink
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
			)
			(VariableNode "$x")
		)
	)
)
