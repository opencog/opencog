(define (do-fuzzy-QA)
    (begin-thread
        (let* ((sent-node (get-current-sent-node))
               (fuz (get-fuzzy-answers sent-node)))
               (State fuzzy-answers (List (map Word (car fuz))))

            ; TODO: Create new ImplicationLinks for this question
        )
    )
)

(define (say . words)
    (if (list? (car words))
        (display (map cog-name (car words)))
        (display (map cog-name words))
    )

    ; Reset evreything
    (State input-utterance no-input-utterance)
    (State canned-rules no-canned-rules)
    (State fuzzy-answers no-fuzzy-answers)
)

(define (answer anchor)
    (let ((ans (cog-chase-link 'StateLink 'ListLink anchor)))
        (if (null? ans)
            '()
            (say (cog-outgoing-set (car ans)))
        )
    )
)
