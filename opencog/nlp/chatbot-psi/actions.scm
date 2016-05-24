(define (do-fuzzy-QA)
    (State fuzzy-qa-search search-started)

    (begin-thread
        (let* ((sent-node (get-input-sent-node))
               (ans (get-fuzzy-answers sent-node)))

            (if (not (null? ans))
                (let ((ans-in-words (List (map Word (car ans)))))
                    (State fuzzy-answers ans-in-words)

                    ; Create a new psi-canned-rule for it
                    (Member
                        (psi-rule
                            (list (SequentialAnd
                                (DefinedPredicate "is-input-utterance?")
                                (Evaluation (GroundedPredicate "scm: did-someone-say-this?") (get-input-word-list))
                            ))
                            (ExecutionOutput (GroundedSchema "scm: say") ans-in-words)
                            (Evaluation (GroundedPredicate "scm: psi-demand-value-maximize") (List sociality (Number "90")))
                            new-rule-tv
                            sociality
                        )
                        canned-rule
                    )
                )
            )
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
