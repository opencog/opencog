;; anaphor is non-reflexive
;; The parse tree structure is:

;;          verb
;;         /    \
;; antecedent   anaphor

;; This antecedent should be rejected

(define filter-#13
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
                (VariableNode "$verb")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$relationship #1")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
            )
            (TypedVariableLink
                (VariableNode "$relationship #2")
                (ListLink
                    (VariableTypeNode "DefinedLinguisticRelationshipNode")
                    (VariableTypeNode "PrepositionalRelationshipNode")
                )
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

                ;; filter
                (NotLink
                    (InheritanceLink
                        (VariableNode "$word-inst-anaphor")
                        (DefinedLinguisticConceptNode "reflexive")
                    )
                )
                (EvaluationLink
                    (VariableNode "$relationship #1")
                    (ListLink
                        (VariableNode "$verb")
                        (VariableNode "$word-inst-antecedent")
                    )
                )
                (EvaluationLink
                    (VariableNode "$relationship #2")
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
)
