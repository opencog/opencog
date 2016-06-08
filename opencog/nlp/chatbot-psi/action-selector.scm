(define (psi-as-chat)
    ; Search rules
    (define (search-rules)
        (let ((rules '()))
            ; For general chat rules
            (set! rules (append rules
                (filter
                    (lambda (r)
                        (if (equal? (psi-satisfiable? r) (stv 1 1)) #t #f))
                    (cog-chase-link 'MemberLink 'ImplicationLink chat-rule)
                )
            ))

            rules
        )
    )

    ; Choose rules
    (define (get-top-ranked-rules)
        (define (weight-and-filter-rules rules)
            (define (weight x)
                (let* ((a-stv (cog-tv x)))
                       ; TODO: Add importance values to the formula when ECAN is ready
                       (* (tv-conf a-stv) (tv-mean a-stv))
                )
            )

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

(psi-set-action-selector
    (ExecutionOutput (GroundedSchema "scm: psi-as-chat") (List))
    sociality
)
