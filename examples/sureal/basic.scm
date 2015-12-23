;
; A simple demo for SuReal
;
; Prior to running this, the RelEx parse server needs to be set up,
; so that the `nlp-parse` call succeeds. The directory containing the
; chatbot has detailed instructions on how to do this.

; Load the needed modules!
(use-modules (opencog)
             (opencog nlp)
             (opencog nlp sureal)
             (opencog nlp chatbot)) ; the chatbot defines nlp-parse

; Populate the contents AtomSpace with various parses.
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
