(define getResults
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$word-inst-antecedent")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word-inst-anaphor")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (ReferenceLink
                (VariableNode "$word-inst-antecedent")
                (VariableNode "$word-inst-anaphor")
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (ReferenceLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "$word-inst-anaphor")
                )
            )
        )
    )
)
