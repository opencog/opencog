;; anaphor is "feminine"
;; antecedent is "masculine"

;; Examples:

;; "Tom ate an apple. She does not like it."
;; "She" should not refer to "Tom"

(define filter-#5
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
                (VariableNode "$word-inst-anaphor")
                (DefinedLinguisticConceptNode "feminine")
            )
            (InheritanceLink
                (VariableNode "$word-inst-antecedent")
                (DefinedLinguisticConceptNode "masculine")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (AnchorNode "Filtered")
        )
    )
)
