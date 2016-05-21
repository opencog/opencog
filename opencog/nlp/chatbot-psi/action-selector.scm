(define (psi-as-chat)
    ; Search rules
    ; TODO: Also need to search more... e.g. QA rules
    (define (search-rules)
        (let ((rules '()))
            ; For searching canned rules
            (cog-chase-link 'MemberLink 'ImplicationLink canned-rule)

            ; For searching QA rules
            (set! rules (append rules (cog-chase-link 'MemberLink 'ImplicationLink qa-rule)))

            ; For searching AIML rules
            ; (let* ((setlink (cog-execute! (Get (State input-utterance (Variable "$s")))))
            ;        (in-utt (car (cog-outgoing-set setlink))))
            ;     (if (equal? (cog-type in-utt) 'ReferenceLink)
            ;         (psi-get-dual-rules
            ;             ; Get ListLink from ReferenceLink
            ;             (car (cog-outgoing-set in-utt))
            ;         )
            ;         '()
            ;     )
            ; )

            rules
        )
    )

    ; Choose rules
    (define (get-top-ranked-rules)
        (define (weight-and-filter-rules rules)
            (define (weight x)
                (let ((a-stv (cog-tv x))
                      ; TODO: Update this
                      (fuz-score 1))
                    ; TODO: Add importance values to the formula when ECAN is ready
                    (* (tv-conf a-stv) (tv-mean a-stv) fuz-score)))

            (define (pick atom lst)
                (cond
                    ((> (weight (car lst)) (weight atom)) lst)
                    ((= (weight (car lst)) (weight atom)) (append lst (list atom)))
                    (else (list atom))))

            (if (null? rules)
                '()
                (delete-duplicates (fold pick (list (car rules)) rules))
            )
        )

        ; TODO: Change this to most-important-weighted-atoms when ECAN is ready
        (weight-and-filter-rules (search-rules))
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
