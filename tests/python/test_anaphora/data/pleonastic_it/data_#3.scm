;; Test #3

;; "It appears that"

;; Expected result:
;; True (It is a pleonastic it)

(LemmaLink
    (WordInstanceNode "it")
    (WordNode "it")
)
(LemmaLink
    (WordInstanceNode "appears")
    (WordNode "appear")
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_expl")
    (ListLink
        (WordInstanceNode "appears")
        (WordInstanceNode "it")
    )
)
(EvaluationLink
    (PrepositionalRelationshipNode "that")
    (ListLink
        (WordInstanceNode "appears")
        (WordInstanceNode "word")
    )
)