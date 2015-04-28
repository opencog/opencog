;;"waitresses and the cook sigh and roll their eyes."

;;anaphor is "their"
;;antecedent is "waitresses"


;; Expected result:
;; True (Conjunction resolution is applied. i.e. "their" refers to "waitresses and the cook")

;; Connection between two clauses
(ListLink
    (AnchorNode "CurrentResolution")
    (WordInstanceNode "their")
    (WordInstanceNode "waitresses")
)
(ListLink
    (AnchorNode "CurrentPronoun")
    (WordInstanceNode "their")
)
(PartOfSpeechLink
    (WordInstanceNode "cook")
    (DefinedLinguisticConceptNode "noun")
)
(PartOfSpeechLink
    (WordInstanceNode "waitresses")
    (DefinedLinguisticConceptNode "noun")
)
(InheritanceLink
    (WordInstanceNode "their")
    (DefinedLinguisticConceptNode "plural")
)
(EvaluationLink
    (PrepositionalRelationshipNode "conj_and")
    (ListLink
        (WordInstanceNode "waitresses")
        (WordInstanceNode "cook")
    )
)