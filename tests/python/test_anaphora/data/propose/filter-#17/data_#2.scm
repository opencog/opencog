;; Test #2

;; anaphor is non-reflexive
;; The parse tree structure is:

;;               verb
;;         to   /    \ by
;;             /      \
;;       antecedent    anaphor

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

(EvaluationLink
    (PrepositionalRelationshipNode "to")
    (ListLink
        (WordInstanceNode "verb")
        (WordInstanceNode "antecedent")
    )
)
(EvaluationLink
    (PrepositionalRelationshipNode "by")
    (ListLink
        (WordInstanceNode "verb")
        (WordInstanceNode "anaphor")
    )
)