;Check if a query is truth query or wh-query 
;--------------------------------------------------------------------------------------------
(define (check_query_type querySentence)
(cog-name 
	(car (cog-chase-link 'InheritanceLink 'ConceptNode 
		(car (cog-chase-link 'InterpretationLink 'InterpretationNode 
			(car (cog-chase-link 'ParseLink 'ParseNode querySentence				
			))
		))
	))
))
;---------------------------------------------------------------------------------------------
;Accept user's quey and user's id as an input
;call "check_query_type" function to check if the query is  'wh' or 'yes-or-no' question
;if query is yes-or-no call 'truth_query_process' else call 'wh_query_process'
;----------------------------------------------------------------------------------------------
(define (process_query query user)
	(define querySentence (car (nlp-parse query)))
	(cond 
		((eq? (check_query_type querySentence) "TruthQuerySpeechAct")
		      (truth_query_process querySentence)
        	((eq? (check_query_type querySentence) "InterrogativeSpeechAct")
		      (wh_query_process querySentence)))

;----------------------------------------------------------------------------------------------
;Use bakward chaning to process the 'yes-or-no' quetion
;depending on the backward chaning result the answer will be generated (using SuRel)
;----------------------------------------------------------------------------------------------
(define (truth_query_process query)
	(define tmp (fAtom query))
	(define bc (cog-bc (cog-new-link 'InheritanceLink 	
				(VariableNode "$x")
				(gdr tmp))))
)
;----------------------------------------------------------------------------------------------
; process wh-question using the fuzzy hyper graph Matcher
;----------------------------------------------------------------------------------------------
(define (wh_query_process query)
;;call the graph matcher 
)




;---------------------------------------------------------------------------------------------
; Used by 'truth_query_process' to find the input for the backward chaining
; Example : if the query is "is mattew student?"
; this function will find 
; Inheritance 
;	"mattew" 
;	"student" 
; from the parse and by substituting mattew with variable $X the backward chaning will be called 
; (cog-bc (InherintanceLink $X "student"))
;---------------------------------------------------------------------------------------------
(define (fAtom query)
	(define x (cog-filter 'EvaluationLink (cog-outgoing-set 
		(car (cog-chase-link 'ReferenceLink 'SetLink 
			(car (cog-chase-link 'InterpretationLink 'InterpretationNode 
				(car (cog-chase-link 'ParseLink 'ParseNode  query))
			))
		))
	)))
(gar (gdr (car (filter cAtom? x)))))
;-----------------------------------------------------------------------------------------------
(define cAtom?
	(lambda (val)
     	(eq? (cog-type (gar (gdr val))) 'InheritanceLink)))













 
