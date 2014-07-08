;; antecedent is a number


(define filter-#15
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
            (TypedVariableLink
                (VariableNode "$verb")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "a number")
                (VariableTypeNode "NumberNode")
            )
        )
        (ImplicationLink
            (AndLink
                ;; Connection between two clauses
                (ListLink
                    (AnchorNode "CurrentResolution")
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

                ;; filter
                (ReferenceLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "a number")
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (AnchorNode "Filtered")
            )
        )
    )
)
