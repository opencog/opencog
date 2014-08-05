;; antecedent is not "noun" with two exceptions
;; "that" can refer to a verb.
;; "enough" can refer to a verb.

;; Example:

;; "Tom saw an apple under a tree, he ate it."
;; While resolving the pronoun "he" or "it", only "Tom", "apple", "tree" are qualified antencedents.
;;
;; Special cases for "that" and "enough":

;; "I will give him some sedative. That should calm him down." -- "give" is a qualified antecedent for "that".

;; "the children kept fighting till they had enough" -- "fighting" is a qualified antecedent for "enough".


(define filter-#1
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
                    (PartOfSpeechLink
                        (VariableNode "$word-inst-antecedent")
                        (DefinedLinguisticConceptNode "noun")
                    )
                )
                (NotLink
                    (LemmaLink
                        (VariableNode "$word-inst-anaphor")
                        (WordNode "that")
                    )
                )
                (NotLink
                    (LemmaLink
                        (VariableNode "$word-inst-anaphor")
                        (WordNode "enough")
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
