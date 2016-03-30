;-----------------------------------------------------------------
; Behavior Rules


(BindLink
        (ListLink
            (ConceptNode "WHEN")
            (ConceptNode "I")
            (GlobNode "$blah")
            (ConceptNode "THEN")
            (GlobNode "$blah2")
        )
        (DefinedPredicateNode "be happy")
)


(BindLink
    (ListLink
        (ConceptNode "WHEN")
        (ConceptNode "THIS")
        (GlobNode "$blah")
        (ConceptNode "YOU")
        (GlobNode "$blah2")
    )
    (DefinedPredicateNode "be happy")
)


(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (ConceptNode "BEAUTIFUL")
    )
    (DefinedPredicateNode "be happy")
)

; TODO OrLinks don't work in antecedent (at top level) with cog-recognize
(BindLink
        (ListLink
            (ConceptNode "YOU")
            (ConceptNode "ARE")
            (GlobNode "$blah")
            (ConceptNode "BEAUTIFUL")
        )
        (DefinedPredicateNode "be happy")
)

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
