;
; Unit testing for nested OrLinks in the pattern matcher.
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
	(ConceptNode "Hank")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Mary")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Phillip")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Milton")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Charlie")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Chayim")
	(ConceptNode "ways and means")
)

(MemberLink
	(ConceptNode "Stuart")
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

;; We should NOT find Hank!
(ListLink
	(MemberLink
		(ConceptNode "Hank")
		(ConceptNode "CEO")
	)
)

(ListLink
	(MemberLink
		(ConceptNode "Mary")
		(ConceptNode "Page")
	)
)

(ListLink
	(MemberLink
		(ConceptNode "Phillip")
		(ConceptNode "Secreatary")
	)
)

(ListLink
	(EvaluationLink
		(PredicateNode "involved")
		(ListLink
			(ConceptNode "Milton")
			(ConceptNode "Business")
		)
	)
)

(ListLink
	(EvaluationLink
		(PredicateNode "involved")
		(ListLink
			(ConceptNode "Charlie")
			(ConceptNode "Industry")
		)
	)
)

(ListLink
	(EvaluationLink
		(PredicateNode "involved")
		(ListLink
			(ConceptNode "Chayim")
			(ConceptNode "Banking")
		)
	)
)

;; Should NOT find Stuart
(ListLink
	(EvaluationLink
		(PredicateNode "involved")
		(ListLink
			(ConceptNode "Stuart")
			(ConceptNode "Diletant")
		)
	)
)

;;; Nested clauses; all connected with a common variable.
(define (nest)
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
						(EvaluationLink
							(PredicateNode "involved")
							(OrLink
								(ListLink
									(VariableNode "$x")
									(ConceptNode "Business")
								)
								(ListLink
									(VariableNode "$x")
									(ConceptNode "Industry")
								)
								(ListLink
									(VariableNode "$x")
									(ConceptNode "Banking")
								)
							)
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)

;; Simple nesting -- Or within Or 
(define (nest-bad)
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
						;;  Note this Or within an Or
						(OrLink
							(MemberLink
								(VariableNode "$x")
								(ConceptNode "Page")
							)
							(MemberLink
								(VariableNode "$x")
								(ConceptNode "Secretary")
							)
						)
					)
				)
			)
			(VariableNode "$x")
		)
	)
)
