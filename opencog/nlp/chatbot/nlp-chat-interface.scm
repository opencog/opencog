;Check if a query is truth query or wh-query 
;--------------------------------------------------------------------------------------------
(define (QueryCheck querySentence)
(cog-chase-link 'InheritanceLink 'ConceptNode
                (car (cog-chase-link 'InterpretationLink 'InterpretationNode
                        (car (cog-chase-link 'ParseLink 'ParseNode querySentence
                        ))
                ))
        )
)


(define (check_query_type querySentence)
(define temp)
(set! temp (QueryCheck querySentence))
(cond

                ((equal? '() temp) "I don't know the type")
                (else (cog-name (car temp)))
))
;---------------------------------------------------------------------------------------------
;Accept user's query and user's id as an input
;call "check_query_type" function to check if the query is  'wh' or 'yes-or-no' question
;if query is yes-or-no call 'truth_query_process' else call 'wh_query_process'
;----------------------------------------------------------------------------------------------
(define (process_query user query)
        (define querySentence)
        (set! querySentence (car (nlp-parse query)))
        (display "Hello ")
        (display user)
        (display ", you said: \"")
        (display query)
        (newline)
        (cond
                       ((equal? (check_query_type querySentence) "TruthQuerySpeechAct")(display "You ask a Truth Query ")
                       ;;(truth_query_process querySentence)
                       (display "I can't process truth query for now"))
                       ((equal? (check_query_type querySentence) "InterrogativeSpeechAct")(display "You made an Interrogative SpeechAct")
                       (wh_query_process query))
                       ((equal? (check_query_type querySentence) "DeclarativeSpeechAct")(display "You made a Declarative SpeechAct"))
                       (else (display "Sorry,I don't know the type"))
         )
)
;----------------------------------------------------------------------------------------------
;Use backward chaning to process 'yes-or-no' question
;depending on the backward chaning generate the answer (using SuRel)
;----------------------------------------------------------------------------------------------
(define (truth_query_process query)
        (define tmp)
        (define bc)
        (set! tmp (fAtom query))
        (set! bc (cog-bc (cog-new-link 'InheritanceLink (VariableNode "$x")(gdr tmp))))
)
;----------------------------------------------------------------------------------------------
; process wh-question using the fuzzy hyper graph Matcher
;----------------------------------------------------------------------------------------------
(define (wh_query_process query)
    (get-similar-sentences query)
)

;---------------------------------------------------------------------------------------------
; Used by 'truth_query_process' to find the input for the backward chaining
; Example : if the query is "is Matthew a student?"
; this function will find 
; Inheritance 
;	"Matthew" 
;	"student" 
; from the parse and by substituting Matthew with variable $X the backward chaning will be called 
; (cog-bc (InherintanceLink $X "student"))
;---------------------------------------------------------------------------------------------
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
;-----------------------------------------------------------------------------------------------
(define cAtom?
	(lambda (val)
     	(eq? (cog-type (gar (gdr val))) 'InheritanceLink)))













 
