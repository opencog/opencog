
; Hack, this is needed for some reason...
(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog) (opencog atom-types))

; Hack, it seems that no one else sets this up ...
(load-r2l-rulebase)

;Check the utterance speech act type
;------------------------------------------------------------------
(define (QueryCheck querySentence)
    (cog-chase-link 'InheritanceLink 'DefinedLinguisticConceptNode
        (car (cog-chase-link 'InterpretationLink 'InterpretationNode
            (car (cog-chase-link 'ParseLink 'ParseNode querySentence))))
    )
)
;-------------------------------------------------------------------
(define (check-query-type querySentence)
    (define temp)
    (set! temp (QueryCheck querySentence))
    (cond
        ((equal? '() temp) "I don't know the speech act type")
        (else (cog-name (car temp))))
)

;-------------------------------------------------------------------
;--------------------------------------------------------------------
(define (process-query user query)
"
  process-query USER QUERY -- accept user's text and generate a reply.

  This is the master opencog chatbot interface; it accepts an utterance
  (in the form of a text string) and generates a reply.  The USER is
  the user-name (currently, the user's IRC nick). The QUERY is the
  string holding what the user said.
"
    (define querySentence (car (nlp-parse query)))

    ;; XXX FIXME -- remove the IRC debug response below.
    (display "Hello ")
    (display user)
    (display ", you said: \"")
    (display query)
    (display "\"")
    (newline)
    ; Call the `check-query-type` function to get the speech act type
    ; of the utterance.  The response processing will be based on the
    ; type of the speech act.
    (cond
        ((equal? (check-query-type querySentence) "TruthQuerySpeechAct")
             (display "You ask a Truth Query ")
        ; (truth_query_process querySentence)
        (display "I can't process truth query for now"))
        ((equal? (check-query-type querySentence) "InterrogativeSpeechAct")
            (display "You made an Interrogative SpeechAct ")
        (wh_query_process querySentence))
        ((equal? (check-query-type querySentence) "DeclarativeSpeechAct")
            (display "You made a Declarative SpeechAct "))
        (else (display "Sorry, I can't identify the speech act type"))
    ))

;--------------------------------------------------------------------
; not working for now ...
;
; need to include the proper rule-base
;
; should also put the sentence's output atom into the focus set
;
; It Use backward chaning to process Truth query
; Depending on the backward chaning generate the answer (using SuRel)
;--------------------------------------------------------------------
(define (truth_query_process query)
    (define tmp)
    (define bc)
    (define rule-base)
    (set! tmp (fAtom query))
    (set! bc (cog-bc
        (cog-new-link 'InheritanceLink (VariableNode "$x") (gdr tmp)) rule-base (SetLink)))
)
;--------------------------------------------------------------------
; Process wh-question using the fuzzy hyper graph Matcher
;--------------------------------------------------------------------
(define (wh_query_process query)
    (define temp)
    (set! temp  (get-answers query))
    (cond
        ((equal? #f (car temp)) "Sorry, I don't know the answer")
        (else (car temp))
))
;-------------------------------------------------------------------
; Used by 'truth_query_process' to find the input for the backward chaining
; This also definitely require ca hange after the backward chaning is completed
;-------------------------------------------------------------------
(define (fAtom querySentence)
    (define x)
    (set! x (cog-filter 'EvaluationLink (cog-outgoing-set
        (car (cog-chase-link 'ReferenceLink 'SetLink
            (car (cog-chase-link 'InterpretationLink 'InterpretationNode
                (car (cog-chase-link 'ParseLink 'ParseNode  querySentence))
            ))
        ))
    )))
(gar (gdr (car (filter cAtom? x)))))
;---------------------------------------------------------------------
(define cAtom?
    (lambda (val)
        (eq? (cog-type (gar (gdr val))) 'InheritanceLink)))

;---------------------------------------------------------------------
