;; Returning the other part of a conjunction if conjunction exists and anaphor is "Plural"

(define getConjunction
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
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
            (PartOfSpeechLink
                (VariableNode "$noun")
                (DefinedLinguisticConceptNode "noun")
            )
            (PartOfSpeechLink
                (VariableNode "$word-inst-antecedent")
                (DefinedLinguisticConceptNode "noun")
            )
            (InheritanceLink
                (VariableNode "$word-inst-anaphor")
                (DefinedLinguisticConceptNode "plural")
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "conj_and")
                (ListLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "$noun")
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$noun")
        )
    )
)
