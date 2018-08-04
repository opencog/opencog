;; Test #1

;; anaphor is non-reflexive
;; The parse tree structure is:

;;             antecedent
;;                 \ "of"
;;                 anaphor

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

(EvaluationLink
    (PrepositionalRelationshipNode "of")
    (ListLink
        (WordInstanceNode "antecedent")
        (WordInstanceNode "anaphor")
    )
)