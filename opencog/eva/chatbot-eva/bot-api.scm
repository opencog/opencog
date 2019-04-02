;
; bot-api.scm
;
; Top-level Eva chatbot API.
; Its a modified fork of ../chatbot/bot-api.scm
;

(use-modules (opencog nlp))

;--------------------------------------------------------------------

;; XXX FIXME this is a cut-n-paste job from process-query

(define-public (grounded-talk USER QUERY)
"
  grounded-talk USER QUERY -- accept USER's text and perform action,
  or maybe generate a reply (replies are currently broken).

  This is a truncated chatbot interface, for use with the robot.
  It accepts an utterance (in the form of a text string) and, if it is
  understood, then the robot performs an action.
  The USER is the user-name  The QUERY is the string holding what the
  user said.
"
    ; nlp-parse returns (SentenceNode "sentence@45c470a6-29...")
    (define sent-node (car (nlp-parse QUERY)))

    ;; XXX FIXME -- remove the IRC debug response below.
    (display "Hello ")
    (display USER)
    (display ", you said: \"")
    (display QUERY)
    (display "\"")
    (newline)

    ; Call the `get-utterance-type` function to get the speech act type
    ; of the utterance.  The response processing will be based on the
    ; type of the speech act.
    ;
    ; XXX Currently, this dispatch is done via scheme code below. The
    ; correct design would use an ImplicationLink to match up the
    ; speech-act type to the processing that would be performed.  This
    ; would allow a single sentence to be interpreted as possibly several
    ; different speech-act types.  It would fit better into the long-term
    ; plan to do all recognition with ImplicationLinks+ure.
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
            ; (wh_query_process sent-node)
            (self-wh-query sent-node)
        )
        ((equal? utr (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))
            (display "You made a Declarative SpeechAct\n")
        )
        ((equal? utr (DefinedLinguisticConceptNode "ImperativeSpeechAct"))
            (display "You made a Imperative SpeechAct\n")
            ; Make the robot do whatever ...
				(imperative-process sent-node)
        )
        (else
            (display "Sorry, I can't identify the speech act type\n")
        )
    )))
