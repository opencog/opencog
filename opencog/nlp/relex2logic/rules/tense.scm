
(define tense
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$tense")
                (TypeNode "DefinedLinguisticConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$lemma")
                (TypeNode "WordNode")
            )
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
