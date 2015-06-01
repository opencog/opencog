;; anaphor is reflexive
;; The parse tree structure is:

;;               verb
;;         to   /    \ by
;;             /      \
;;       antecedent    anaphor


;; This antecedent should be rejected

;; Examples:

;; "Jacob went to the party by himself."
;; "himself" should not refer to "party"


(define filter-#17
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
                (VariableNode "$verb")
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
                (DefinedLinguisticConceptNode "reflexive")
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "to")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$word-inst-antecedent")
                )
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "by")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$word-inst-anaphor")
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (AnchorNode "Filtered")
        )
    )
)
