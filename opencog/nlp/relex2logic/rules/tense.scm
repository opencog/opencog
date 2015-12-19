
(define tense
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$tense" "DefinedLinguisticConceptNode")
			(var-decl "$lemma" "WordNode")
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (PartOfSpeechLink
                (VariableNode "$verb")
                (DefinedLinguisticConceptNode "verb")
            )
            (TenseLink
                (VariableNode "$verb")
                (VariableNode "$tense")
            )
            (LemmaLink
                (VariableNode "$verb")
                (VariableNode "$lemma")
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
