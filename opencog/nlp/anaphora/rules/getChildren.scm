(define getChildren
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (VariableTypeNode "DefinedLinguisticRelationshipNode")
            )
            (VariableNode "$target")
            (VariableNode "$child")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "currentTarget")
                    (VariableNode "$target")
                )
                (EvaluationLink
                    (VariableNode "$relationshipNode")
                    (ListLink
                        (VariableNode "$target")
                        (VariableNode "$child")
                    )
                )
            )
            (ListLink
                (AnchorNode "ChildrenAnchorNode")
                (VariableNode "$child")
            )
        )
    )
)
