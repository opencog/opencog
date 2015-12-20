
(define det
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$noun" "WordInstanceNode")
			(var-decl "$det" "WordInstanceNode")
			(var-decl "$noun-lemma" "WordNode")
			(var-decl "$det-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$noun" "$a-parse")
			(word-in-parse "$det" "$a-parse")
			(dependency "_det" "$noun" "$det")
			(word-feat "$noun" "definite")
			(word-lemma "$det" "$det-lemma")
			(word-lemma "$noun" "$noun-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: det-rule")
			(ListLink
				(VariableNode "$noun-lemma")
				(VariableNode "$noun")
				(VariableNode "$det-lemma")
			)
		)
	)
)
