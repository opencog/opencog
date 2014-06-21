(define getChildren
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (VariableTypeNode "LinkGrammarRelationshipNode")
            )
            (VariableNode "$target")
            (VariableNode "$child")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "targetAnchor")
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
