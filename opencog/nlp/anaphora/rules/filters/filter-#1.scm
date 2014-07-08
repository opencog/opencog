;; antecedent is not "noun"

(define filter-#1
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
                (NotLink
                    (PartOfSpeechLink
                        (VariableNode "$word-inst-antecedent")
                        (DefinedLinguisticConceptNode "noun")
                    )
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (AnchorNode "Filtered")
            )
        )
    )
)
