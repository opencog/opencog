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

(MemberLink
	(ConceptNode "Tom")
	(ConceptNode "Senator")
)

(MemberLink
	(ConceptNode "Joe")
	(ConceptNode "Representative")
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
			(VariableNode "$x")
		)
	)
)
