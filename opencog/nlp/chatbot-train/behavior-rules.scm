
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

(BindLink
    (ListLink
        (ConceptNode "YOU")
        (ConceptNode "ARE")
        (GlobNode "$blah")
        (ConceptNode "BEAUTIFUL")
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
