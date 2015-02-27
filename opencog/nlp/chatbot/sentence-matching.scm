; Returns one or more sentences that are structurally similar to the input one.
;
; For example:
;    (get-similar-sentences "What did Pete eat?")
;
; Possible result: 
;    (Pete ate apples .)
;
(define (get-similar-sentences input-sentence)
    ; Generate sentences from each of the R2L-SetLinks
    (define (generate-sentences r2l-setlinks) (if (> (length r2l-setlinks) 0) (map sureal r2l-setlinks) '()))

    (begin
        ; Delete identical sentences from the return set
        (delete-duplicates
            ; Use SuReal to generate sentences from their corresponding SetLinks
            (generate-sentences
                ; Search for any similar SetLinks in the atomspace
                (cog-fuzzy
                    ; Get the R2L SetLink of the input sentence
                    (car (cog-chase-link 'ReferenceLink 'SetLink
                        (car (cog-chase-link 'InterpretationLink 'InterpretationNode
                            (car (cog-chase-link 'ParseLink 'ParseNode
                                (car (nlp-parse input-sentence))
                            ))
                        ))
                    ))
                )
            )
        )
    )
)
