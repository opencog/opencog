; Referenced from https://github.com/opencog/relex/blob/master/data/relex2logic-rules.txt
;
; It was supposed to be the order of speech-act-rule execution in the original
; design, and only one speech act will be generated per sentence.
; Since we now switch to use the forward chainer for executing the R2L rules and
; at the moment there is no control over the order of rule execution nor which
; to execute, it is possible that more than one speech act will be generated for
; the same sentence. In that case the same order as listed below will be used to
; decide which one to keep or delete.
; 
(define speech-act-orders
    '(
        ("InterrogativeSpeechAct" . 1)
        ("TruthQuerySpeechAct" . 2)
        ("ImperativeSpeechAct" . 2)
        ("DeclarativeSpeechAct" . 3)
    )
)
