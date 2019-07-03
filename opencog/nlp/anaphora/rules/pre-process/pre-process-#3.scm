;; target is "enough" and there does not exist "of" after it.

(define pre-process-#3
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
            (ListLink
                (AnchorNode "CurrentTarget")
                (VariableNode "$target")
            )
            (LemmaLink
                (VariableNode "$target")
                (WordNode "enough")
            )
            (AbsentLink
                (EvaluationLink
                    (PrepositionalRelationshipNode "of")
                    (ListLink
                        (VariableNode "$target")
                        (VariableNode "$word")
                    )
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$target")
        )
    )
)
