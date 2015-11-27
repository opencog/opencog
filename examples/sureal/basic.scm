; A simple demo for SuReal

; The contents of the AtomSpace
(nlp-parse "Madonna wrote the song.")
(nlp-parse "John and Batman saved the city.")
(nlp-parse "The dog ate that smelly cake.")
(nlp-parse "Jackson recorded the album in 1985.")
(nlp-parse "This pig can fly.")

; Generate a new sentence by running SuReal
; Expected result: "John ate the pig ."
(sureal
    (SetLink
        (EvaluationLink
            (PredicateNode "ate")
            (ListLink
                (ConceptNode "John")
                (ConceptNode "pig")
            )
        )
    )
)
