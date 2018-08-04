;; Test #1

;; "It means that"

;; Expected result:
;; True (It is a pleonastic it)

(LemmaLink
    (WordInstanceNode "it")
    (WordNode "it")
)
(LemmaLink
    (WordInstanceNode "means")
    (WordNode "mean")
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_subj")
    (ListLink
        (WordInstanceNode "means")
        (WordInstanceNode "it")
    )
)
(EvaluationLink
    (PrepositionalRelationshipNode "that")
    (ListLink
        (WordInstanceNode "means")
        (WordInstanceNode "word")
    )
)
