(define anaphora-resolution
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
                    (AnchorNode "Recent Unresolved references")
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
                (NotLink
                    (InheritanceLink
                        (VariableNode "$word-inst-antecedent")
                        (DefinedLinguisticConceptNode "pronoun")
                    )
                )
                (InheritanceLink
                    (VariableNode "$anaphor-number")
                    ;;(DefinedNumberConceptNode "NumberNode") DEBUG
                    (ConceptNode "NumberNodeConcept")
                )
                (InheritanceLink
                    (VariableNode "$antecedent-number-number")
                    ;;(DefinedNumberConceptNode "NumberNodeConcept") DEBUG
                    (ConceptNode "NumberNodeConcept")
                )
                ;;(WordNumberLink  DEBUG)
                (ListLink
                    (VariableNode "$word-inst-anaphor")
                    (VariableNode "$anaphor-number")
                )
                ;;(WordNumberLink  DEBUG)
                (ListLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "$antecedent-number")
                )
                ;;(GreaterThanLink DEBUG)
                (ListLink
                    (VariableNode "$anaphor-number")
                    (VariableNode "$antecedent-number")
                )
            )
            (ReferenceLink
                (VariableNode "$word-inst-antecedent")
                (VariableNode "$word-inst-anaphor")
            )
        )
    )
)



