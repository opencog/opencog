;; Returning children associated with current target.

(define getChildren
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$relationshipNode")
                (TypeChoice
                    (TypeNode "DefinedLinguisticRelationshipNode")
                    (TypeNode "PrepositionalRelationshipNode")
                )
            )
            (VariableNode "$target")
            (VariableNode "$child")
        )
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
