;; anaphor is non-reflexive
;; The parse tree structure is:

;;          verb
;;         /    \
;; antecedent    anaphor

;; This antecedent should be rejected

;; This antecedent should be rejected

;; Examples:

;; "Tom likes him."
;; "him" should not refer to "Tom"

(define filter-#11
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
            (AbsentLink
                (InheritanceLink
                    (VariableNode "$word-inst-anaphor")
                    (DefinedLinguisticConceptNode "reflexive")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$word-inst-antecedent")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$verb")
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
