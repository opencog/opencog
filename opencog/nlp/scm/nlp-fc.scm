; This loads all the rules into the cogserver shell. This assumes that the
; cogserver is started from in-source build directory.
(define (load-r2l-rulebase)
    (load-scm-from-file "../opencog/nlp/relex2logic/loader/load-rules.scm")
    (load-scm-from-file
        "../opencog/nlp/relex2logic/loader/gen-r2l-en-rulebase.scm")
)

(define (load-speech-act-orders)
    (load-scm-from-file "../opencog/nlp/relex2logic/speech-act-orders.scm")
)

; Run forward chaining on the sentence
(define (nlp-fc sent)
    (let* ((parse-node (car (sentence-get-parses (nlp-parse sent))))
           (interp-node (gen-interp-node (cog-name parse-node)))
           (interp-link (gen-interp-link interp-node parse-node))
           (results (cog-outgoing-set (car (run-fc parse-node interp-link))))
           (speech-acts '())
           (result-contents '()))

        (for-each (lambda (r)
            (let ((s1 (cog-outgoing-set r)))
                (for-each (lambda (x)
                    (let ((s2 (cog-outgoing-set x)))
                        (for-each (lambda(y)
                            (if (equal? (cog-type y) 'InheritanceLink)
                                ; Extract the speech-act-InheritanceLinks.
                                ; Assumming it is in the form of:
                                ;    (InheritanceLink
                                ;        (InterpretationNode "sentence@123")
                                ;        (ConceptNode "SomeSpeechAct")
                                ;    )
                                (if (and (equal? (car (cog-outgoing-set y)) interp-node)
                                         (not (equal?
                                             (string-contains
                                                 (cog-name (cadr (cog-outgoing-set y)))
                                                 "SpeechAct"
                                             )
                                         '#f))
                                     )
                                    (set! speech-acts (append speech-acts (list y)))
                                )

                                ; Store everything else in the result-contents
                                (set! result-contents (append result-contents (list y)))
                            )
                        ) s2)
                    )
                ) s1)
            )
        ) results)

        ; Sort the speech acts according to their orders
        (sort speech-acts
            (lambda (x y) (< (cdr (assoc (cog-name (cadr (cog-outgoing-set x))) speech-act-orders))
                             (cdr (assoc (cog-name (cadr (cog-outgoing-set y))) speech-act-orders))))
        )

        ; Construct a ReferenceLink as the output
        (ReferenceLink
            interp-node

            ; The function in the SetLink returns a list of outputs that
            ; are the results of the evaluation of the relex-to-logic functions,
            ; on the relex-opencog-outputs.
            (SetLink
                (delete-duplicates result-contents)
                (if (>= (length speech-acts) 1)
                    (car speech-acts)
                    '()
                )
            )
        )

        (AtTimeLink
            ; FIXME: maybe opencog's internal time octime should be used. Will do for
            ; now assuming a single instance deals with a single conversation.
            (TimeNode (number->string (current-time)))
            interp-node
            (TimeDomainNode "Dialogue-System")
        )

        ; Delete any extra speech acts that might have been generated
        (if (> (length speech-acts) 1) (map cog-delete-recursive (cdr speech-acts)))
    )
    #t
)

(define (gen-interp-node parse-name)
    (InterpretationNode (string-append parse-name "_interpretation_$X"))
)

(define (gen-interp-link interp-node parse-node)
    (InterpretationLink
       interp-node
       parse-node
    )
)

(define (run-fc parse-node interp-link)
    (list (cog-fc
        (SetLink (parse-get-relex-outputs parse-node) interp-link)
        r2l-rules
    ))
)
