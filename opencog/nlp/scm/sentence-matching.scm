; Check if the input is an actual sentence or a SentenceNode,
; parse and return the corresponding SentenceNode if it is a sentence
;
(define (sentence-node input)
    (if (cog-atom? input)
        (if (equal? 'SentenceNode (cog-type input))
            input
            (display "Please input a SentenceNode only")
        )
        (car (nlp-parse input))
    )
)

; Calls the fuzzy pattern matcher to find similar sentences from the Atomspace.
;
; Accepts either an actual sentence or a SentenceNode as the input.
; Returns one or more similar sentences.
;
; For example:
;    (get-similar-sentences "What did Pete eat?")
; OR:
;    (get-similar-sentences (SentenceNode "sentence@123"))
;
; Possible result: 
;    (Pete ate apples .)
;
(define (get-similar-sentences input-sentence)
    (main-ss (sentence-node input-sentence) '())
)

; Similar to get-similar-sentence but it's mainly for question answering purpose.
;
; Find answers (i.e., similar sentences that share some keyword) from the
; Atomspace by using the fuzzy pattern matcher. By default, it excludes
; sentences with TruthQuerySpeechAct and InterrogativeSpeechAct.
;
; Accepts either an actual sentence or a SentenceNode as the input. Also accepts
; an optional argument "exclude-list" which is a list of atoms that we don't want
; them to exist in the hypergraphs of the answers. By default they are
; (DefinedLinguisticConceptNode "TruthQuerySpeechAct") and (DefinedLinguisticConceptNode "InterrogativeSpeechAct").
;
; Returns one or more sentences -- the answers.
;
(define get-answers
    (case-lambda
        ((question) (main-ss (sentence-node question) (list (DefinedLinguisticConceptNode "TruthQuerySpeechAct")
                                                            (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))))
        ((question exclude-list) (main-ss (sentence-node question) exclude-list))
    )
)

; The main function for finding similar sentences
; Returns one or more sentences that are similar to the input one but
; contains no atoms that are listed in the exclude-list
;
(define (main-ss sentence-node exclude-list)
    ; Generate sentences from each of the R2L-SetLinks
    (define (generate-sentences r2l-setlinks) (if (> (length r2l-setlinks) 0) (map sureal r2l-setlinks) '()))

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
                            (car (cog-chase-link 'ParseLink 'ParseNode sentence-node))
                        ))
                    ))
                    'SetLink
                    exclude-list
                ))
            )
        )
    )
)

