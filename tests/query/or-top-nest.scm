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
(MemberLink
	(ConceptNode "Tom")
	(ConceptNode "Senator")
)

(MemberLink
	(ConceptNode "Joe")
	(ConceptNode "Representative")
)

;; We should NOT find Hank!
(MemberLink
	(ConceptNode "Hank")
	(ConceptNode "CEO")
)

(MemberLink
	(ConceptNode "Mary")
	(ConceptNode "Page")
)

(MemberLink
	(ConceptNode "Phillip")
	(ConceptNode "Secretary")
)

(EvaluationLink
	(PredicateNode "involved")
	(ListLink
		(ConceptNode "Milton")
		(ConceptNode "Business")
	)
)

(EvaluationLink
	(PredicateNode "involved")
	(ListLink
		(ConceptNode "Charlie")
		(ConceptNode "Industry")
	)
)

(EvaluationLink
	(PredicateNode "involved")
	(ListLink
		(ConceptNode "Chayim")
		(ConceptNode "Banking")
	)
)

;; Should NOT find Stuart
(EvaluationLink
	(PredicateNode "involved")
	(ListLink
		(ConceptNode "Stuart")
		(ConceptNode "Diletant")
	)
)

;;; Nested clauses; all connected with a common variable.
(define (top-nest)
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
			(VariableNode "$x")
		)
	)
)

;; Simple nesting -- Or within Or 
(define (top-nest-bad)
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
			(VariableNode "$x")
		)
	)
)
