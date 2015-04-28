;; Test #1

;; anaphor is "plural"
;; antecedent is "singular"

;; Expected result:
;; Rejection

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
    (DefinedLinguisticConceptNode "plural")
)
(InheritanceLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "singular")
)