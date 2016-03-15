; This rule is for negation of predicates, as in "You do not smell good, sir."
; (AN June 2015)


(define neg
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$pred" "WordInstanceNode")
			(var-decl "$pred-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$pred" "$a-parse")
			(word-lemma  "$pred" "$pred-lemma")
			(word-feat "$pred" "negative")
		)
		(ExecutionOutputLink
		   (GroundedSchemaNode "scm: negative-rule")
			(ListLink
				(VariableNode "$pred-lemma")
				(VariableNode "$pred")
			)
		)
	)
)
