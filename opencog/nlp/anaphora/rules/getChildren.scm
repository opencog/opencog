;; Returning children associated with current target.

(define getChildren
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (VariableNode "$target")
            (VariableNode "$child")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
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
                (AnchorNode "CurrentResult")
                (VariableNode "$child")
            )
        )
    )
)
