
; Hack, this is needed for some reason...
(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog)
             (opencog atom-types)
             (opencog rule-engine)
             (opencog nlp)
             (opencog nlp microplanning)
             (opencog nlp relex2logic))

; hack -- these need to be modules of thier own.
(load "../scm/nlp-utils.scm")
(load "../scm/sentence-matching.scm")

;------------------------------------------------------------------
(define (get-utterance-type sent)
"
  get-utterance-type SENT -- Check the utterance speech act type

  Expect SENT to be (SentenceNode \"sentence@45c470a6-29...\")
  Will return (DefinedLinguisticConceptNode ACT) where ACT is
  one of DeclarativeSpeechAct, InterrogativeSpeechAct,
  TruthQuerySpeechAct, etc...
"
    ; parse will be (ParseNode "sentence@a6_parse_0")
    (define parse (car (cog-chase-link 'ParseLink 'ParseNode sent)))
    ; interp will be (InterpretationNode "sentence@a610_interpretation_$X")

    (define interp (car
        (cog-chase-link 'InterpretationLink 'InterpretationNode parse)))

    ; act-type will be (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
    (define act-type (cog-chase-link
        'InheritanceLink 'DefinedLinguisticConceptNode interp))

    ; Return string holding the speech act type.
    (cond
        ((equal? '() act-type) "Unknown speech act type")
        (else (cog-name (car act-type))))
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
    ; nlp-parse returns (SentenceNode "sentence@45c470a6-29...")
    (define querySentence (car (nlp-parse query)))

    ;; XXX FIXME -- remove the IRC debug response below.
    (display "Hello ")
    (display user)
    (display ", you said: \"")
    (display query)
    (display "\"")
    (newline)
    ; Call the `get-utterance-type` function to get the speech act type
    ; of the utterance.  The response processing will be based on the
    ; type of the speech act.
    (cond
        ((equal? (get-utterance-type querySentence) "TruthQuerySpeechAct")
             (display "You ask a Truth Query ")
        ; (truth_query_process querySentence)
        (display "I can't process truth query for now"))
        ((equal? (get-utterance-type querySentence) "InterrogativeSpeechAct")
            (display "You made an Interrogative SpeechAct ")
        (wh_query_process querySentence))
        ((equal? (get-utterance-type querySentence) "DeclarativeSpeechAct")
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
