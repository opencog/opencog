
(define gender
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$word" "WordInstanceNode")
			(var-decl "$gtype" "DefinedLinguisticConceptNode")
			(var-decl "$lemma" "WordNode")
        )
        (AndLink
			(word-in-parse "$word" "$a-parse")
            (InheritanceLink
                (VariableNode "$word")
                (VariableNode "$gtype")
            )
            (InheritanceLink
                (VariableNode "$word")
                (DefinedLinguisticConceptNode "person")
            )
            (LemmaLink
                (VariableNode "$word")
                (VariableNode "$lemma")
            )
            (OrLink
                (EqualLink
                    (VariableNode "$gtype")
                    (DefinedLinguisticRelationshipNode "masculine"))
                (EqualLink
                    (VariableNode "$gtype")
                    (DefinedLinguisticRelationshipNode "feminine"))
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
