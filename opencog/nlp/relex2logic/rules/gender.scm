
(define gender
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$word" "WordInstanceNode")
			(var-decl "$lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$word" "$a-parse")
			(word-lemma "$word" "$lemma")
			(word-feat "$word" "person")
			(ChoiceLink
				(word-feat "$word" "masculine")
				(word-feat "$word" "feminine")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: gender-rule")
			(ListLink
				(VariableNode "$lemma")
				(VariableNode "$word")
				(VariableNode "$gtype")
			)
		)
	)
)
