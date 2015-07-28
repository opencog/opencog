; The get-similar-sentences interface
(define get-similar-sentences
	(case-lambda
		((s) (main-search s '()))
		((s rl) (main-search s rl))
	)
)

; The main get-similar-sentences function.
; Returns one or more sentences that are structurally similar to the input one.
;
; For example:
;    (get-similar-sentences "What did Pete eat?")
; OR:
;    (get-similar-sentences (SentenceNode "sentence@123"))
;
; Possible result: 
;    (Pete ate apples .)
;
(define (main-search input-sentence reject-list)
    ; Make sure it is at least a SetLink before sending it to sureal for sentence generation
    (define (to-sureal a-link) (if (equal? 'SetLink (cog-type a-link)) (sureal a-link) #f))

    ; Generate sentences from each of the R2L-SetLinks
    (define (generate-sentences r2l-setlinks) (if (> (length r2l-setlinks) 0) (map to-sureal r2l-setlinks) '()))

    ; Check if the input-sentence is an actual sentence or a SentenceNode,
    ; parse and return the corresponding SentenceNode if it is a sentence
    (define (sentence-node)
        (if (cog-atom? input-sentence)
            (if (equal? 'SentenceNode (cog-type input-sentence))
                input-sentence
                (display "Please input a SentenceNode only")
            )
            (car (nlp-parse input-sentence))
        )
    )

    (begin
        ; Delete identical sentences from the return set
        (delete-duplicates
            ; Use SuReal to generate sentences from their corresponding SetLinks
            (generate-sentences
                ; Search for any similar SetLinks in the atomspace
                (cog-outgoing-set (cog-fuzzy-match
                    ; Get the R2L SetLink of the input sentence
                    (car (cog-chase-link 'ReferenceLink 'SetLink
                        (car (cog-chase-link 'InterpretationLink 'InterpretationNode
                            (car (cog-chase-link 'ParseLink 'ParseNode
                                (sentence-node)
                            ))
                        ))
                    ))
                    reject-list
                ))
            )
        )
    )
)

