;Check the utterance speech act type
;------------------------------------------------------------------
(define (QueryCheck querySentence)
    (cog-chase-link 'InheritanceLink 'ConceptNode
        (car (cog-chase-link 'InterpretationLink 'InterpretationNode
            (car (cog-chase-link 'ParseLink 'ParseNode querySentence))))
    )
)
;-------------------------------------------------------------------
(define (check_query_type querySentence)
    (define temp)
    (set! temp (QueryCheck querySentence))
    (cond
        ((equal? '() temp) "I don't know the speech act type")
        (else (cog-name (car temp))))
)

;-------------------------------------------------------------------
; Accept user's ID and utterance as input
; Call "check_query_type" function to get the speech act type of the utterance
; based on the type of the speech act a processing functions will be called
;--------------------------------------------------------------------
(define (process_query user query)
    (define querySentence)
    (set! querySentence (car (nlp-parse query)))
    (display "Hello ")
    (display user)
    (display ", you said: \"")
    (display query)
    (newline)
    (cond
        ((equal? (check_query_type querySentence) "TruthQuerySpeechAct")
             (display "You ask a Truth Query ")
        ; (truth_query_process querySentence)
        (display "I can't process truth query for now"))
        ((equal? (check_query_type querySentence) "InterrogativeSpeechAct")
            (display "You made an Interrogative SpeechAct ")
        (wh_query_process query))
        ((equal? (check_query_type querySentence) "DeclarativeSpeechAct")
            (display "You made a Declarative SpeechAct "))
        (else (display "Sorry,I don't know the type"))
    ))

;--------------------------------------------------------------------
; not working for now ... after the work of backward chaning is
; completed it may need some fix
; It Use backward chaning to process Truth query
; Depending on the backward chaning generate the answer (using SuRel)
;--------------------------------------------------------------------
(define (truth_query_process query)
    (define tmp)
    (define bc)
    (set! tmp (fAtom query))
    (set! bc (cog-bc
        (cog-new-link 'InheritanceLink (VariableNode "$x") (gdr tmp))))
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
