;; Test #4

;; "It is a dog"

;; Expected result:
;; False (It is not a pleonastic it)

(LemmaLink
    (WordInstanceNode "it")
    (WordNode "it")
)
(EvaluationLink
  (DefinedLinguisticRelationshipNode "_subj")
  (ListLink
    (WordInstanceNode "is@17b44503-095b-4a91-837c-59a9de147da6")
    (WordInstanceNode "it")
  )
)
(EvaluationLink
  (DefinedLinguisticRelationshipNode "_obj")
  (ListLink
    (WordInstanceNode "is@17b44503-095b-4a91-837c-59a9de147da6")
    (WordInstanceNode "dog@6bc39bf4-2d39-4d7e-ab4f-1d6b998b7ef0")
  )
)
