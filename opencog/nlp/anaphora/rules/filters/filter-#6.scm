;; anaphor is "neuter"
;; antecedent is "masculine"

;; Examples:

;; "Tom ate an apple. It does not like it."
;; "It" should not refer to "Tom"


(define filter-#6
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
                (DefinedLinguisticConceptNode "neuter")
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
