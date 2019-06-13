;; antecedent's number > anaphora's number

;; Examples:

;; "Tom saw an apple under a tree, he ate a banana."
;; While resolving the pronoun "he", "banana" should not be an antecedent.

(define filter-#3
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
            (TypedVariableLink
                (VariableNode "$antecedent-number")
                (TypeNode "NumberNode")
            )
            (TypedVariableLink
                (VariableNode "$anaphor-number")
                (TypeNode "NumberNode")
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
            (VariableNode "$word-inst-antecedent")
        )
    )
)
