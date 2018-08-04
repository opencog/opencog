
(define gender
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$word" "WordInstanceNode")
			(var-decl "$lemma" "WordNode")
			(var-decl "$gtype" "DefinedLinguisticConceptNode")
		)
		(AndLink
			(word-in-parse "$word" "$a-parse")
			(word-lemma "$word" "$lemma")
			(word-feat "$word" "person")
			(InheritanceLink (Variable "$word") (Variable "$gtype"))
			(OrLink
				(EqualLink (Variable "$gtype") (DefinedLinguisticConceptNode "masculine"))
				(EqualLink (Variable "$gtype") (DefinedLinguisticConceptNode "feminine"))
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
