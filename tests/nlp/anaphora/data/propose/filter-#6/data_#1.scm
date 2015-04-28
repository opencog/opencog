;; Test #1

;; anaphor is "neuter"
;; antecedent is "masculine"

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
    (DefinedLinguisticConceptNode "neuter")
)
(InheritanceLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "masculine")
)