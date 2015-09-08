; This loads all the rules into the cogserver shell. This assumes that the
; cogserver is started from in-source build directory.
(define (load-r2l-rulebase)
    (load-scm-from-file "../opencog/nlp/relex2logic/loader/load-rules.scm")
    (load-scm-from-file
        "../opencog/nlp/relex2logic/loader/gen-r2l-en-rulebase.scm")
)

; Run forward chaining on the sentence
(define (nlp-fc sent)
    (let* ((parse-node (car (sentence-get-parses (nlp-parse sent))))
           (interp-node (gen-interp-node (cog-name parse-node)))
           (interp-link (gen-interp-link interp-node parse-node))
           (results (cog-outgoing-set (car (run-fc parse-node interp-link))))
           (result-contents '()))

        (for-each (lambda (r)
            (let ((s1 (cog-outgoing-set r)))
                (for-each (lambda (x)
                    (let ((s2 (cog-outgoing-set x)))
                        (for-each (lambda(y)
                            ; Extract and store everything in the result-contents
                            (set! result-contents (append result-contents (list y)))
                        ) s2)
                    )
                ) s1)
            )
        ) results)

        ; Construct a ReferenceLink as the output
        (ReferenceLink
            interp-node

            ; The function in the SetLink returns a list of outputs that
            ; are the results of the evaluation of the relex-to-logic functions,
            ; on the relex-opencog-outputs.
            (SetLink (delete-duplicates result-contents))
        )

        (AtTimeLink
            ; FIXME: maybe opencog's internal time octime should be used. Will do for
            ; now assuming a single instance deals with a single conversation.
            (TimeNode (number->string (current-time)))
            interp-node
            (TimeDomainNode "Dialogue-System")
        )
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
