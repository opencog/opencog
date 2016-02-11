;
; process-query.scm
;
; Top-level Eva chatbot handling.
; Its a modified fork of ../chatbot/process-query.scm
;

(use-modules (opencog nlp) (opencog nlp fuzzy))

;-------------------------------------------------------------------
;--------------------------------------------------------------------
(define (wh_query_process query)
"
  Process wh-question in a cascade of attempts by different subsystems.
  FIrst, try to see if its a self-awarenes question; if not, then use
  the fuzzy hypergraph Matcher.

  QUERY should be a SentenceNode.
"
	; self-wh-query provided by self-model.scm
	(define ans (self-wh-query query))
	(cond
		((not (equal? '() ans)) (string-join (car ans)))
		(else (let
			; get-answers provided by (opencog nlp fuzzy)
			((temp (get-answers query)))
			(cond
				((equal? '() temp) "Sorry, I don't know the answer.")
				(else (string-join (car temp)))))
		)
	)
)
;--------------------------------------------------------------------
(define-public (process-query user query)
"
  process-query USER QUERY -- accept user's text and generate a reply.

  This is the master opencog chatbot interface; it accepts an utterance
  (in the form of a text string) and generates a reply.  The USER is
  the user-name (currently, the user's IRC nick). The QUERY is the
  string holding what the user said.
"
    ; nlp-parse returns (SentenceNode "sentence@45c470a6-29...")
    (define sent-node (car (nlp-parse query)))

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
    (let* ((gutr (sentence-get-utterance-type sent-node))
           (utr (if (equal? '() gutr) '() (car gutr)))
        )
    (cond
        ((equal? utr (DefinedLinguisticConceptNode "TruthQuerySpeechAct"))
            (display "You asked a Truth Query\n")
            ; (truth_query_process sent-node)
            (display "I can't process truth query for now\n")
        )
        ((equal? utr (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
            (display "You made an Interrogative SpeechAct\n")
            (wh_query_process sent-node)
        )
        ((equal? utr (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))
            (display "You made a Declarative SpeechAct\n")
            ; XXX Use AIML here to say something snarky.
        )
        ((equal? utr (DefinedLinguisticConceptNode "ImperativeSpeechAct"))
            (display "You made a Imperative SpeechAct\n")
            ; Make the robot do whatever ...
				(imperative-process sent-node)
            ; XXX Use AIML here to say something snarky.
        )
        (else
            (display "Sorry, I can't identify the speech act type\n")
            ; XXX Use AIML here to say something snarky.
        )
    )))

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
;-------------------------------------------------------------------
; Used by 'truth_query_process' to find the input for the backward
; chaining.
; XXX FIXME This also definitely requires change after the backward
; chaining is completed.
;-------------------------------------------------------------------
(define (fAtom querySentence)
    (define x (cog-filter 'EvaluationLink (cog-outgoing-set
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
