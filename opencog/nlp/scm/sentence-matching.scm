
; XXX this needs to be part of some module, maybe a fuzzy-matcher
; module, or something like that.

(use-modules (opencog)
             (opencog nlp)
             (opencog nlp sureal)
             (opencog nlp microplanning))

(define-public (get-answers question)
"
  Find answers (i.e., similar sentences that share some keyword) from
  the Atomspace by using the fuzzy pattern matcher. By default, it
  excludes sentences with TruthQuerySpeechAct and InterrogativeSpeechAct.

  Accepts either an actual sentence or a SentenceNode as the input.
  Also accepts an optional argument \"exclude-list\" which is a list of
  atoms that we don't want them to exist in the hypergraphs of the
  answers. By default they are
     (DefinedLinguisticConceptNode \"TruthQuerySpeechAct\")
  and (DefinedLinguisticConceptNode \"InterrogativeSpeechAct\").

  Returns one or more sentences -- the answers.

  For example:
     (get-answers \"What did Pete eat?\")
  OR:
     (get-answers (SentenceNode \"sentence@123\"))

  Possible result:
     (Pete ate apples .)
"
    ; Check if the input is an actual sentence or a SentenceNode,
    ; parse and return the corresponding SentenceNode if it is a sentence
    (define (process-q input)
        (if (cog-atom? input)
            (if (equal? 'SentenceNode (cog-type input))
                input
                #f
            )
            (car (nlp-parse input))
        )
    )

    (let ((sent-node (process-q question))
          (ex-list (list (DefinedLinguisticConceptNode "TruthQuerySpeechAct")
                         (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))))
        (if sent-node
            (sent-matching sent-node ex-list)
            (display "Please input a SentenceNode only")
        )
    )
)

(define-public (sent-matching sent-node exclude-list)
"
  The main function for finding similar sentences
  Returns one or more sentences that are similar to the input one but
  contains no atoms that are listed in the exclude-list
"
    ; Generate sentences from each of the R2L-SetLinks
    ; (define (generate-sentences r2l-setlinks) (if (> (length r2l-setlinks) 0) (map sureal r2l-setlinks) '()))

    ; Generate sentences for each of the SetLinks found by the fuzzy matcher
    ; TODO: May need to filter out some of the contents of the SetLinks
    ; before sending each of them to Microplanner
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
            ; Send each of the SetLinks found by the fuzzy matcher to
            ; Microplanner to see if they are good
            (let ((m-results (microplanning (SequentialAndLink (cog-outgoing-set r)) (get-speech-act r) *default_chunks_option* #f)))
                ; Don't send it to SuReal in case it's not good
                ; (i.e. Microplanner returns #f)
                (if m-results
                    (append-map
                        ; Send each of the SetLinks returned by
                        ; Microplanning to SuReal for sentence generation
                        (lambda (m) (sureal (car m)))
                        m-results
                    )
                    '()
                )
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
