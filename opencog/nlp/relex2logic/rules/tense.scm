
(define tense
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$tense" "DefinedLinguisticConceptNode")
			(var-decl "$lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-lemma "$verb" "$lemma")
			(word-pos "$verb" "verb")
			(TenseLink
				(VariableNode "$verb")
				(VariableNode "$tense")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: tense-rule")
			(ListLink
				(VariableNode "$lemma")
				(VariableNode "$verb")
				(VariableNode "$tense")
			)
		)
	)
)
