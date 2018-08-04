;; Test #1

;; anaphor is "it"
;; antecedent is "plural"

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

(ReferenceLink
    (WordInstanceNode "anaphor")
    (WordNode "it")
)
(InheritanceLink
    (WordInstanceNode "antecedent")
    (DefinedLinguisticConceptNode "plural")
)