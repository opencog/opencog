;-----------------------------------------------------------------
; Behavior Rules

(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (ConceptNode "BEAUTIFUL")
    )
    (DefinedPredicateNode "be happy")
)

; TODO OrLinks working in antecedent (at top level) with cog-recognize
(BindLink
        (ListLink
            (ConceptNode "YOU")
            (ConceptNode "ARE")
            (GlobNode "$blah")
            (ConceptNode "BEAUTIFUL")
        )
        (DefinedPredicateNode "be happy")
)

; TODO: This one only works currently with single word matches to the globs
; due to https://github.com/opencog/atomspace/issues/724
(BindLink
        (ListLink
            (ConceptNode "YOU")
            (ConceptNode "ARE")
            (GlobNode "$blah")
            (ConceptNode "BEAUTIFUL")
            (GlobNode "$blah2")
        )
        (DefinedPredicateNode "be happy")
)


(BindLink
    (ListLink
        (ConceptNode "BE")
        (ConceptNode "HAPPY")
    )
    (DefinedPredicateNode "be happy"))


(BindLink
  (ListLink
    (ConceptNode "ARE")
    (ConceptNode "YOU")
    (ConceptNode "BORED")
  )
  (DefinedPredicateNode "yawn")
)
