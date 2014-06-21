(define propose
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
                    (AnchorNode "currentProposal")
                    (VariableNode "$word-inst-anaphor")
                )
                (PartOfSpeechLink
                    (VariableNode "$word-inst-antecedent")
                    (DefinedLinguisticConceptNode "noun")
                )
                (PartOfSpeechLink
                    (VariableNode "$word-inst-anaphor")
                    (DefinedLinguisticConceptNode "noun")
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
            )
            (ReferenceLink
                (VariableNode "$word-inst-antecedent")
                (VariableNode "$word-inst-anaphor")
            )
        )
    )
)
