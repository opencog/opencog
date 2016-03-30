; This rule is for "how of degree questions" such as "How fast can you run?"
; (AN June 2015)

(define howdeg-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$pred" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$qVar" "$a-parse")
			(word-in-parse "$pred" "$a-parse")
			(dependency "_%howdeg" "$pred" "$qVar")
			(word-feat "$qVar" "how_much")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howdeg-q-rule")
			(ListLink
				(VariableNode "$pred")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-howdeg-q-rule pred)
	(howdegQ-rule (cog-name (word-inst-get-lemma pred)) (cog-name pred))
)
