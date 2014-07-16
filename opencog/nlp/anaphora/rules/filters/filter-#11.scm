;; anaphor is "plural"
;; antecedent is "singular" and there is no "conj_and" link such that
;; (EvaluationLink
;;      (PrepositionalRelationshipNode "conj_and")
;;             (ListLink
;;                   (VariableNode "$word-inst-antecedent")
                     (VariableNode "$noun")
;;             )
;;       )
;; )

(define filter-#11
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
                (VariableNode "$noun")
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
                (InheritanceLink
                    (VariableNode "$word-inst-anaphor")
                    (DefinedLinguisticConceptNode "plural")
                )
                (InheritanceLink
                    (VariableNode "$word-inst-antecedent")
                    (DefinedLinguisticConceptNode "singular")
                )
                (PartOfSpeechLink
                    (VariableNode "$noun")
                    (DefinedLinguisticConceptNode "noun")
                )
                (NotLink
                    (EvaluationLink
                        (PrepositionalRelationshipNode "conj_and")
                        (ListLink
                            (VariableNode "$word-inst-antecedent")
                            (VariableNode "$noun")
                        )
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
