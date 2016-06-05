(define (do-fuzzy-QA)
    (State fuzzy-qa-search search-started)

    (begin-thread
        (let* ((sent-node (get-input-sent-node))
               (ans (get-fuzzy-answers sent-node #:do-microplanning #f)))

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

(define (do-aiml-search)
    (State aiml-search search-started)

    (begin-thread
        (State aiml-replies (aiml-get-response-wl (get-input-word-list)))
    )
)

(define (say . words)
    (if (list? (car words))
        (display (map cog-name (car words)))
        (display (map cog-name words))
    )

    (reset-all-states)
)

(define (reply anchor)
    (let ((ans (cog-chase-link 'StateLink 'ListLink anchor)))
        (if (null? ans)
            '()
            (say (cog-outgoing-set (car ans)))
        )
    )
)

(define (reset-all-states)
    (State input-utterance no-input-utterance)
    (State canned-rules default-state)
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa-search default-state)
)
