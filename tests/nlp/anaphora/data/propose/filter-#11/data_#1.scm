;; Test #1

;; anaphor is non-reflexive
;; The parse tree structure is:

;;          verb
;;         /    \
;; antecedent   anaphor

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
    (DefinedLinguisticRelationshipNode "_subj")
    (ListLink
        (WordInstanceNode "verb")
        (WordInstanceNode "antecedent")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "verb")
        (WordInstanceNode "anaphor")
    )
)