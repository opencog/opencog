
(define gender
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$gtype")
                (TypeNode "DefinedLinguisticConceptNode")
            )
            (TypedVariableLink
                (VariableNode "$lemma")
                (TypeNode "WordNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$word")
                (VariableNode "$a-parse")
            )
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
