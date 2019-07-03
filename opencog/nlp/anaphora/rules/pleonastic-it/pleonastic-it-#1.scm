;; "It means that"

(define pleonastic-it-#1
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$means")
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
                (VariableNode "$means")
                (WordNode "mean")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$means")
                    (VariableNode "$target")
                )
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "that")
                (ListLink
                    (VariableNode "$means")
                    (VariableNode "$word")
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$target")
        )
    )
)
