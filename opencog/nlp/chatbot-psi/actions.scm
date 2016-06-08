(define (do-fuzzy-QA)
    (State fuzzy-qa-search search-started)

    (begin-thread
        (let* ((sent-node (get-input-sent-node))
               (ans (get-fuzzy-answers sent-node #:do-microplanning #f)))
            (if (not (null? ans))
                (let ((ans-in-words (List (map Word ans))))
                    (State fuzzy-answers ans-in-words)

                    ; TODO: Create a new psi-rule for this QA in the OpenCog AIML format
                )
            )
        )
    )
)

(define (do-aiml-search)
    (State aiml-search search-started)

    (begin-thread
        (let ((aiml-resp (aiml-get-response-wl (get-input-word-list))))
            ; No result if it's a ListLink with arity 0
            (if (equal? (cog-arity aiml-resp) 0)
                (State aiml-replies no-aiml-reply)
                (State aiml-replies aiml-resp)
            )
        )
    )
)

(define (say . words)
    (define utterance "")

    (if (list? (car words))
        (set! utterance (string-join (map cog-name (car words))))
        (set! utterance (string-join (map cog-name words)))
    )

    (display utterance)

    ; For sending out the chatbot response via the grounded predicate defined
    ; in ros-behavior-scripting
    (catch #t
        (lambda ()
            (cog-evaluate! (Evaluation (GroundedPredicate "py: say_text") (List (Node utterance))))
        )
        (lambda (key . parameters)
            (display "\nWarning: Failed to call \"py: say_text\" to send out the message.\n")
        )
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
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa-search default-state)
)
