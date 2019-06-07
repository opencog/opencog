;; "It appears that"

(define pleonastic-it-#3
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$appears")
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
                (VariableNode "$appears")
                (WordNode "appear")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_expl")
                (ListLink
                    (VariableNode "$appears")
                    (VariableNode "$target")
                )
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "that")
                (ListLink
                    (VariableNode "$appears")
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
