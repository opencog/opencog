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
                (TypedVariableLink
                    (VariableNode "$antecedent-number")
                    (VariableTypeNode "NumberNode")
                )
                (TypedVariableLink
                    (VariableNode "$anaphor-number")
                    (VariableTypeNode "NumberNode")
                )
            )
            (ImplicationLink
                (AndLink
                    (ListLink
                        (AnchorNode "CurrentPronoun")
                        (VariableNode "$word-inst-anaphor")
                    )
                    (ListLink
                        (AnchorNode "CurrentProposal")
                        (VariableNode "$word-inst-antecedent")
                    )
                    (PartOfSpeechLink
                        (VariableNode "$word-inst-antecedent")
                        (DefinedLinguisticConceptNode "noun")
                    )
                    (InheritanceLink
                        (VariableNode "$word-inst-anaphor")
                        (DefinedLinguisticConceptNode "pronoun")
                    )
                    (WordSequenceLink
                        (VariableNode "$word-inst-anaphor")
                        (VariableNode "$anaphor-number")
                    )
                    (WordSequenceLink
                        (VariableNode "$word-inst-antecedent")
                        (VariableNode "$antecedent-number")
                    )
                    (EvaluationLink
                        (GroundedPredicateNode "c++:greater")
                        (ListLink
                            (VariableNode "$anaphor-number")
                            (VariableNode "$antecedent-number")
                        )
                    )

        ;;(NotLink
            ;;(InheritanceLink
            ;;    (VariableNode "$word-inst-antecedent")
           ;;     (DefinedLinguisticConceptNode "masculine")
           ;; )
       ;; )

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
