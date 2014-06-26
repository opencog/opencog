(define (filterGenerator filter)
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
                (AndLink
                    ;; Connection between two clauses
                    (ListLink
                         (AnchorNode "CurrentRe")
                         (VariableNode "$word-inst-anaphor")
                         (VariableNode "$word-inst-antecedent")
                    )
                    (ListLink
                        (AnchorNode "CurrentPronoun")
                        (VariableNode "$word-inst-anaphor")
                    )
                    (ListLink
                        (AnchorNode "CurrentProposal")
                        (VariableNode "$word-inst-antecedent")
                    )

                )
                (ListLink
                    (AnchorNode "CurrentResult")
                    (ListLink
                        (VariableNode "$word-inst-antecedent")
                        (VariableNode "$word-inst-anaphor")
                    )
                )
            )
        )
)
