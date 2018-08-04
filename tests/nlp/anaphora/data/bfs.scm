;; The following testing data is a dependency tree used for test_bfs unit test
;; Tree structure:
;;                       a
;;                      / \
;;                     b   c
;;                    /|\   \
;;                   d e f   g

;; Expected traversal order:
;; (a,b,c,d,e,f,g)

(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "a")
        (WordInstanceNode "b")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "a")
        (WordInstanceNode "c")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "b")
        (WordInstanceNode "d")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "b")
        (WordInstanceNode "e")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "b")
        (WordInstanceNode "f")
    )
)
(EvaluationLink
    (DefinedLinguisticRelationshipNode "_obj")
    (ListLink
        (WordInstanceNode "c")
        (WordInstanceNode "g")
    )
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