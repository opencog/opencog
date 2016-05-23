(define (psi-as-chat)
    ; Search rules
    (define (search-rules)
        (let ((rules '()))
            ; For searching canned rules
            (let ((cr (cog-chase-link 'StateLink 'ListLink canned-rules)))
                (if (not (null? cr))
                    (set! rules (append rules (cog-outgoing-set (car cr))))
                )
            )

            ; For searching QA rules
            (set! rules (append rules
                (filter
                    (lambda (r)
                        (if (equal? (psi-satisfiable? r) (stv 1 1)) #t #f))
                    (cog-chase-link 'MemberLink 'ImplicationLink qa-rule)
                )
            ))

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
