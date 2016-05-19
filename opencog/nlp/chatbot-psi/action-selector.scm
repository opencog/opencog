(define (psi-as-chat)
    ; Search rules
    (define (search-rules)
        (let* ((setlink (cog-execute! (Get (State input-utterance (Variable "$s")))))
               (in-utt (car (cog-outgoing-set setlink))))
            (if (equal? (cog-type in-utt) 'ReferenceLink)
                (psi-get-dual-rules
                    ; Get ListLink from ReferenceLink
                    (car (cog-outgoing-set in-utt))
                )
                '()
            )
        )
    )

    ; Choose rules
    (define (get-top-ranked-rules)
        ; TODO: Change this to most-important-weighted-atoms when ECAN is ready
        (most-weighted-atoms (search-rules))
    )

    ; Pick rules if there are still more than one
    (let ((rules (get-top-ranked-rules)))
        (if (null? rules)
            (Set)
            (Set (list-ref rules (random (length rules))))
        )
    )
)

(psi-action-selector-set!
    (psi-add-action-selector
        (ExecutionOutput (GroundedSchema "scm: psi-as-chat") (List))
        "chat"
    )
)
