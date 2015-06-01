;; anaphor is "it"
;; antecedent is "plural"


;; Examples:

;; "There are some apples in the corner. It is beautiful."
;; "It" should not refer to "apples"

(define filter-#10
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
            (ReferenceLink
                (VariableNode "$word-inst-anaphor")
                (WordNode "it")
            )
            (InheritanceLink
                (VariableNode "$word-inst-antecedent")
                (DefinedLinguisticConceptNode "plural")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (AnchorNode "Filtered")
        )
    )
)
