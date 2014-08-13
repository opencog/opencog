;; antecedent's number > anaphora's number

;; Examples:

;; "Tom saw an apple under a tree, he ate a banana."
;; While resolving the pronoun "he", "banana" should not be an antecedent.

(define filter-#3
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
                (WordSequenceLink
                    (VariableNode "$word-inst-anaphor")
                    (VariableNode "$anaphor-number")
                )
                (WordSequenceLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "$antecedent-number")
                )

                ;; filter
                (EvaluationLink
                    (GroundedPredicateNode "c++:greater")
                    (ListLink
                        (VariableNode "$antecedent-number")
                        (VariableNode "$anaphor-number")
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
