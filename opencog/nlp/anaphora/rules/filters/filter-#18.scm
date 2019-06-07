;; antecedent should not be the anphor itself.

;; Examples:

;; This one should be obvious.

(define filter-#18
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$word-inst-antecedent")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word-inst-anaphor")
                (TypeNode "WordInstanceNode")
            )
        )
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
            (ListLink
                (AnchorNode "CurrentPronoun")
                (VariableNode "$word-inst-antecedent")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$word-inst-antecedent")
        )
    )
)
