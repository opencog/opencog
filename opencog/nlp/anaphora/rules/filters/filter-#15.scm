;; anaphor is non-reflexive
;; The parse tree structure is:

;;             antecedent
;;                 \ "of"
;;                 anaphor

;; This antecedent should be rejected

;; Examples:

;; "John's portrait of him."
;; "him" should not refer to "portrait "

(define filter-#15
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
            (AbsentLink
                (InheritanceLink
                    (VariableNode "$word-inst-anaphor")
                    (DefinedLinguisticConceptNode "reflexive")
                )
            )
            (EvaluationLink
                (PrepositionalRelationshipNode "of")
                (ListLink
                    (VariableNode "$word-inst-antecedent")
                    (VariableNode "$word-inst-anaphor")
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$word-inst-antecedent")
        )
    )
)
