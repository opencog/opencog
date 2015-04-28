;; Test #2

;; anaphor is reflexive
;; The parse tree structure is:

;;          verb
;;         /    \
;; antecedent   anaphor

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
    (DefinedLinguisticConceptNode "reflexive")
)
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