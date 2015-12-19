
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
			(lemma-of-word "$verb" "$lemma")
			(PartOfSpeechLink
				(VariableNode "$verb")
				(DefinedLinguisticConceptNode "verb")
			)
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
