(define (say . words)
    (if (list? (car words))
        (display (map cog-name (car words)))
        (display (map cog-name words))
    )

    ; TODO: Reset the QA-related StateLinks if it's a question
    (State input-utterance no-input-utterance)
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
