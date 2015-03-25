;
; greater-compute.scm
;
; Similar to the greater-than unit test, but requires that execution
; of programs to occur during the proccess
;

; Raw data to populate the search space
;
; TRC rower Ken Gates set a world record in 2015--
; an under-3-minute 1K time at the Erg Rodeo
; a week before a 1st place finish at the CRASH-B's
(EvaluationLink
	(PredicateNode "ergs")
	(ListLink
		(ConceptNode "Ken")
		(NumberNode  10)
	)
)

(EvaluationLink
	(PredicateNode "ergs")
	(ListLink
		(ConceptNode "Peter")
		(NumberNode  8)
	)
)

(EvaluationLink
	(PredicateNode "ergs")
	(ListLink
		(ConceptNode "Linas")
		(NumberNode  4)
	)
)

(EvaluationLink
	(PredicateNode "ergs")
	(ListLink
		(ConceptNode "Joe Novice")
		(NumberNode  1)
	)
)

; Function that takes the square root of a numeric values
(define (eff x)
	(NumberNode (sqrt (string->number (cog-name x))))
)

; Function that does some computation
; It associates person with the cube of the quantity.
; (Announces who is attending CRASH-B's this year.)
(define (crash-b who quant)
	(EvaluationLink
		(PredicateNode "power")
		(ListLink
			who
			(cog-execute! (TimesLink quant quant quant))
		)
	)
)

(define (threshold)
	(BindLink
		(SignatureLink
			(VariableNode "$who")
			(VariableNode "$how_much")
		)
		(ImplicationLink
			(AndLink
				(EvaluationLink
					(PredicateNode "ergs")
					(ListLink
						(VariableNode "$who")
						(VariableNode "$how_much")
					)
				)
				;; GreaterThan links are TV-valued when evaluated
				;; Evaluate the inequality
				;; 142 < (1.4 * $how_much * $how_much) + f($how_much)
				;; true: 142 < (1.4 * 10 * 10) + sqrt(10)
				;; false: 142 < (1.4 * 8 * 8) + sqrt(8)
				(GreaterThanLink
					(PlusLink
						(TimesLink
							(NumberNode 1.4)
							(VariableNode "$how_much")
							(VariableNode "$how_much")
						)
						(ExecutionOutputLink
							(GroundedSchemaNode "scm: eff")
							(ListLink
								(VariableNode "$how_much")
							)
						)
					)
					(NumberNode 142)
				)
			)
			(ExecutionOutputLink
				(GroundedSchemaNode "scm: crash-b")
				(ListLink
					(VariableNode "$who")
					(VariableNode "$how_much")
				)
			)
		)
	)
)
