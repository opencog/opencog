(define (do-fuzzy-QA)
    (State fuzzy-qa-search search-started)

    (begin-thread
        (let* ((sent-node (get-input-sent-node))
               (fuz-ans (get-fuzzy-answers sent-node #:do-microplanning #f)))
            (if (null? fuz-ans)
                (State fuzzy-answers no-fuzzy-answers)

                ; Fuzzy matcher may return more than one answers that has the
                ; same score, randomly pick one of them if so
                (let* ((ans (list (list-ref fuz-ans (random (length fuz-ans)))))
                       (ans-in-words (List (map Word (string-split (car ans) #\ )))))
                    (State fuzzy-answers ans-in-words)

                    ; TODO: Create a new psi-rule for this QA in the OpenCog AIML format
                )
            )

(newline)
(display "----------> Fuzzy answers:\n")
(display (car (append (cog-chase-link 'StateLink 'ListLink fuzzy-answers)
    (cog-chase-link 'StateLink 'ConceptNode fuzzy-answers))))
(newline)

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

(newline)
(display "----------> AIML replies:\n")
(display (car (append (cog-chase-link 'StateLink 'ListLink aiml-replies)
    (cog-chase-link 'StateLink 'ConceptNode aiml-replies))))
(newline)

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
            (display "\n(Warning: Failed to call \"py: say_text\" to send out the message.)\n")
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
