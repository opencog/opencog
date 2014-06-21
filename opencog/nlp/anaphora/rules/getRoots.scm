(define getRoots
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (VariableTypeNode "LinkGrammarRelationshipNode")
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
                (AnchorNode "Roots")
                (VariableNode "$root")
            )
        )
    )
)
