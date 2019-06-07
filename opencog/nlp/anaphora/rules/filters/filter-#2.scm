;; antecedent is "pronoun"

;; Example:

;; "He is good at math. She is good at math as well."
;; "She" cannot refer to "He".

(define filter-#2
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
            (InheritanceLink
                (VariableNode "$word-inst-antecedent")
                (DefinedLinguisticConceptNode "pronoun")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$word-inst-antecedent")
        )
    )
)
