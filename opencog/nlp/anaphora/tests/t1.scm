(define test
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
                         (AnchorNode "CurrentRe")
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
                    ;; a particular filter
                    (AndLink
                        (NotLink
                            (InheritanceLink
                                (VariableNode "$word-inst-antecedent")
                                (DefinedLinguisticConceptNode "noun")
                            )
                        )
                        (InheritanceLink
                            (VariableNode "$word-inst-anaphor")
                            (DefinedLinguisticConceptNode "pronoun")
                        )
                    )
                )
                (ListLink
                    (AnchorNode "CurrentResult")
                    (ListLink
                        (VariableNode "$word-inst-antecedent")
                        (VariableNode "$word-inst-anaphor")
                    )
                )
            )
        )
)
