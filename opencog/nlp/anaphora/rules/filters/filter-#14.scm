;; anaphor is non-reflexive
;; The parse tree structure is:

;;          verb
;;         /    \
;; antecedent    noun
;;                 \ "of"
;;                 anaphor

;; This antecedent should be rejected

;; Examples:

;; "John saw a picture of him."
;; "him" should not refer to "John"

(define filter-#14
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$noun")
                (VariableTypeNode "WordInstanceNode")
            )
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
                        (VariableNode "$noun")
                    )
                )
                (EvaluationLink
                    (PrepositionalRelationshipNode "of")
                    (ListLink
                        (VariableNode "$noun")
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
