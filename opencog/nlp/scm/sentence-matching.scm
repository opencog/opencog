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
(define (main-ss sent-node exclude-list)
    ; Generate sentences from each of the R2L-SetLinks
    ; (define (generate-sentences r2l-setlinks) (if (> (length r2l-setlinks) 0) (map sureal r2l-setlinks) '()))

    ; Generate sentences for each of the SetLinks found by the fuzzy matcher
    ; TODO: May need to filter out some of the contents of the SetLinks before sending each of them to Microplanner
    (define (generate-sentences setlinks)
        ; Find the speech act from the SetLink and use it for Microplanning
        (define (get-speech-act setlink)
            (let* ((speech-act-node-name
                        (filter (lambda (name)
                            (if (string-suffix? "SpeechAct" name) #t #f))
                                (map cog-name (cog-filter 'DefinedLinguisticConceptNode (cog-get-all-nodes setlink))))))

                ; If no speech act was found, return "declarative" as default
                (if (> (length speech-act-node-name) 0)
                    (string-downcase (substring (car speech-act-node-name) 0 (string-contains (car speech-act-node-name) "SpeechAct")))
                    "declarative"
                )
            )
        )

        (append-map (lambda (r)
            ; Send each of the SetLinks returned by Microplanning to SuReal for sentence generation
            (append-map
                ; Don't send it to SuReal in case it's not good (i.e. Microplanner returns #f)
                (lambda (m) (if m (sureal m)))
                ; Send each of the SetLinks found by the fuzzy matcher to Microplanner to see if they are good
                (car (microplanning (SequentialAndLink (cog-outgoing-set r)) (get-speech-act r) *default_chunks_option* #f))
            ))
            setlinks
        )
    )

    (begin
        ; Delete identical sentences from the return set
        (delete-duplicates
            ; Use Mircoplanner and SuReal to generate sentences from the SetLinks found
            (generate-sentences
                ; Search for any similar SetLinks in the atomspace
                (cog-outgoing-set (cog-fuzzy-match
                    ; Get the R2L SetLink of the input sentence
                    (car (cog-chase-link 'ReferenceLink 'SetLink
                        (car (cog-chase-link 'InterpretationLink 'InterpretationNode
                            (car (cog-chase-link 'ParseLink 'ParseNode sent-node))
                        ))
                    ))
                    'SetLink
                    exclude-list
                ))
            )
        )
    )
)

