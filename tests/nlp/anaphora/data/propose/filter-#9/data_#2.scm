;; Test #2

;; anaphor is "singular"
;; antecedent is "singular"

;; Expected result:
;; Acceptance

;; Connection between two clauses

(ListLink
    (AnchorNode "CurrentResolution")
    (WordInstanceNode "anaphor")
    (WordInstanceNode "antecedent")
)
(ListLink
    (AnchorNode "CurrentPronoun")
    (WordInstanceNode "anaphor")
)
(ListLink
    (AnchorNode "CurrentProposal")
    (WordInstanceNode "antecedent")
)
;; filter tests

(InheritanceLink
    (WordInstanceNode "anaphor")
    (DefinedLinguisticConceptNode "singular")
)
(InheritanceLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "singular")
)