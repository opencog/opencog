;; Test #2

;; anaphor is "neuter"
;; antecedent is "neuter"

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
    (DefinedLinguisticConceptNode "neuter")
)
(InheritanceLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "neuter")
)