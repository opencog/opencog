;; Test #5

;; "It looks like a dog"

;; Expected result:
;; False (It is not a pleonastic it)

(LemmaLink
    (WordInstanceNode "it")
    (WordNode "it")
)
(EvaluationLink
  (DefinedLinguisticRelationshipNode "_obj")
  (ListLink
    (WordInstanceNode "like@0236eff7-e8e5-4de5-8ec3-99dc5782b7e2")
    (WordInstanceNode "dog@978f75d0-2c6a-40ad-800f-06231f27f523")
  )
)

(EvaluationLink
  (DefinedLinguisticRelationshipNode "_subj")
  (ListLink
    (WordInstanceNode "looks@56656f21-599a-49a0-b411-397af39f9813")
    (WordInstanceNode "it")
  )
)

(EvaluationLink
  (PrepositionalRelationshipNode "like")
  (ListLink
    (WordInstanceNode "looks@56656f21-599a-49a0-b411-397af39f9813")
    (WordInstanceNode "dog@978f75d0-2c6a-40ad-800f-06231f27f523")
  )
)

(InheritanceLink
  (WordInstanceNode "like@0236eff7-e8e5-4de5-8ec3-99dc5782b7e2")
  (DefinedLinguisticConceptNode ".p")
)

(InheritanceLink
  (WordInstanceNode "it")
  (DefinedLinguisticConceptNode "definite")
)

(InheritanceLink
  (WordInstanceNode "it")
  (DefinedLinguisticConceptNode "neuter")
)

(InheritanceLink
  (WordInstanceNode "it")
  (DefinedLinguisticConceptNode "pronoun")
)

(InheritanceLink
  (WordInstanceNode "dog@978f75d0-2c6a-40ad-800f-06231f27f523")
  (DefinedLinguisticConceptNode "singular")
)

(InheritanceLink
  (WordInstanceNode "dog@978f75d0-2c6a-40ad-800f-06231f27f523")
  (DefinedLinguisticConceptNode ".n")
)

(InheritanceLink
  (WordInstanceNode "looks@56656f21-599a-49a0-b411-397af39f9813")
  (DefinedLinguisticConceptNode ".v")
)

(InheritanceLink
  (WordInstanceNode "looks@56656f21-599a-49a0-b411-397af39f9813")
  (DefinedLinguisticConceptNode "present")
)
