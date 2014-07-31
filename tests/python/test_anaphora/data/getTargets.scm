;; The following testing data is a set of words used for test_getTaregts unit test
;; Input data:
;; Name:    a         b              c                d             e          f              g                   h                 j
;; Tag:  pronoun     .n          .n-"enough"        numeric        .v        prep        "enough" +"of"     pronoun+"of"       numeric+"of"

;; Expected result:
;; (a,c,d)

(InheritanceLink
    (WordInstanceNode "a")
    (DefinedLinguisticConceptNode "pronoun")
)

(InheritanceLink
    (WordInstanceNode "b")
    (DefinedLinguisticConceptNode ".n")
)

(InheritanceLink
    (WordInstanceNode "c")
    (DefinedLinguisticConceptNode ".n")
)

(LemmaLink
    (WordInstanceNode "c")
    (WordNode "enough")
)

(InheritanceLink
    (WordInstanceNode "d")
    (DefinedLinguisticConceptNode "numeric")
)

(InheritanceLink
    (WordInstanceNode "e")
    (DefinedLinguisticConceptNode ".v")
)

(InheritanceLink
    (WordInstanceNode "f")
    (DefinedLinguisticConceptNode "prep")
)

(InheritanceLink
    (WordInstanceNode "g")
    (DefinedLinguisticConceptNode ".n")
)

(LemmaLink
    (WordInstanceNode "g")
    (WordNode "enough")
)

(EvaluationLink
    (PrepositionalRelationshipNode "of")
    (ListLink
        (WordInstanceNode "g")
        (VariableNode "a word")
    )
)

(InheritanceLink
    (WordInstanceNode "h")
    (DefinedLinguisticConceptNode "pronoun")
)

(EvaluationLink
    (PrepositionalRelationshipNode "of")
    (ListLink
        (WordInstanceNode "h")
        (VariableNode "a word")
    )
)

(InheritanceLink
    (WordInstanceNode "j")
    (DefinedLinguisticConceptNode "numeric")
)

(EvaluationLink
    (PrepositionalRelationshipNode "of")
    (ListLink
        (WordInstanceNode "j")
        (VariableNode "a word")
    )
)