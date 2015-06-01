;; antecedent is a number

;; Examples:

;; "Twenty cops are hiding in the corner, they are doing something secretly."
;; "they" should not refer to "Twenty"


(define filter-#13
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
            (TypedVariableLink
                (VariableNode "a number")
                (TypeNode "NumberNode")
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
