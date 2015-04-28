;; The following testing data is a set of words used for test_getWords unit test
;; Input data:
;; Name:    a         b              c                d             e          f              g                   h                 j
;; Tag:  pronoun     .n          .n-"enough"        numeric        .v        prep        "enough" +"of"     pronoun+"of"       numeric+"of"

;; Expected result:
;; (a,b,c,d,e,f,g,h,j)

(WordInstanceLink
    (WordInstanceNode "a")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "b")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "c")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "d")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "e")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "f")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "g")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "h")
    (ParseNode "a parse node")
)
(WordInstanceLink
    (WordInstanceNode "j")
    (ParseNode "a parse node")
)

(WordSequenceLink
    (WordInstanceNode "a")
    (NumberNode "1")
)
(WordSequenceLink
    (WordInstanceNode "b")
    (NumberNode "2")
)
(WordSequenceLink
    (WordInstanceNode "c")
    (NumberNode "3")
)
(WordSequenceLink
    (WordInstanceNode "d")
    (NumberNode "4")
)
(WordSequenceLink
    (WordInstanceNode "e")
    (NumberNode "5")
)
(WordSequenceLink
    (WordInstanceNode "f")
    (NumberNode "6")
)
(WordSequenceLink
    (WordInstanceNode "g")
    (NumberNode "7")
)
(WordSequenceLink
    (WordInstanceNode "h")
    (NumberNode "8")
)
(WordSequenceLink
    (WordInstanceNode "j")
    (NumberNode "9")
)
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