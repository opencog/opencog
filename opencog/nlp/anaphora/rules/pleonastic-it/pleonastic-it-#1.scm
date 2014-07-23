;; "It means that"

(define pleonastic-it-#1
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$means")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
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
                (AnchorNode "Matched")
            )
        )
    )
)
