; This is for how-of-quantity questions, such as "How much did that bong cost?"
; (AN June 2015)

(define howquant-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$noun" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$qVar" "$a-parse")
			(word-in-parse "$noun" "$a-parse")
			(dependency "_quantity" "$noun" "$qVar")
			(word-feat "$qVar" "how_much")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howquant-q-rule")
			(ListLink
				(VariableNode "$noun")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-howquant-q-rule noun)
	(howquantQ-rule (cog-name (word-inst-get-lemma noun)) (cog-name noun))
)
