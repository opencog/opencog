(define getRoots
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$top relationship")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$other relationship")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$parent")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$root")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$child")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
               (EvaluationLink
                    (VariableNode "$top relationship")
                    (ListLink
                        (VariableNode "$root")
                        (VariableNode "$child")
                    )
                )
                (NotLink
                    (EvaluationLink
                        (VariableNode "$other relationship")
                        (ListLink
                            (VariableNode "$parent")
                            (VariableNode "$root")
                        )
                    )
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (VariableNode "$root")
            )
        )
    )
)
