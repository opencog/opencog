;; anaphor is "singular"
;; antecedent is "plural"

;; Examples:

;; "There are some cops in the corner. He does not like it."
;; "He" should not refer to "cops"


(define filter-#8
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
                (DefinedLinguisticConceptNode "singular")
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
