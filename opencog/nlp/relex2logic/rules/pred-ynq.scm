; This rule is for yes/no questions with verbs other than "be"
; such as "Did you sleep well?"
; (AN June 2015)


(define pred-ynq
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-feat "$verb" "truth-query")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-pred-ynq-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-pred-ynq-rule verb)
	(pred-ynQ-rule (cog-name (word-inst-get-lemma  verb)) (cog-name verb))
)
