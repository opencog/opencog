;; Test #1

;; antecedent's number > anaphora's number

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

(WordSequenceLink
    (WordInstanceNode "antecedent")
    (NumberNode "3")
)

(WordSequenceLink
    (WordInstanceNode "anaphor")
    (NumberNode "1")
)