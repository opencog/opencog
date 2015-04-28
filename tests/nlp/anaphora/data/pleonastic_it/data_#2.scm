;; Test #2

;; "It seems that"

;; Expected result:
;; True (It is a pleonastic it)

(LemmaLink
    (WordInstanceNode "it")
    (WordNode "it")
)
(LemmaLink
    (WordInstanceNode "seems")
    (WordNode "seem")
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_expl")
    (ListLink
        (WordInstanceNode "seems")
        (WordInstanceNode "it")
    )
)
(EvaluationLink
    (PrepositionalRelationshipNode "that")
    (ListLink
        (WordInstanceNode "seems")
        (WordInstanceNode "word")
    )
)
