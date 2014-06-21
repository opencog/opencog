(define getRoots
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (VariableTypeNode "DefinedLinguisticRelationshipNode")
            )
            (TypedVariableLink
                (VariableNode "$parent")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$root")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (NotLink
                (EvaluationLink
                    (VariableNode "$relationshipNode")
                    (ListLink
                        (VariableNode "$parent")
                        (VariableNode "$root")
                    )
                )
            )
            (ListLink
                (AnchorNode "currentTarget")
                (VariableNode "$root")
            )
        )
    )
)
