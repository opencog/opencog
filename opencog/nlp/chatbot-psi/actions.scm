(use-modules (ice-9 threads))
(use-modules (opencog))

;-------------------------------------------------------------------------------
; Useful functions for the actions

; For handling things return by the fuzzy matcher
(define (pick-and-generate list-of-results)
    (if (equal? (length list-of-results) 0)
        '()
        (let* (; TODO: Should be bias according to the score
               (picked (list-ref list-of-results (random (length list-of-results))))
               ; TODO: Should use gen-sentences when new microplanner is ready
               (generated (sureal (gar picked))))
            (if (null? generated)
                ; Do it again if the chosen one can't be used to generate a sentence
                (pick-and-generate (delete! generated list-of-results))
                generated
            )
        )
    )
)

;-------------------------------------------------------------------------------

(define (call-chatbot-eva)
    (State chatbot-eva sent-to-chatbot-eva)

    (begin-thread
        (imperative-process (get-input-sent-node))
    )
)

(define (do-fuzzy-QA)
    (State fuzzy-qa search-started)

    (begin-thread
        (let ((fuz-ans (get-fuzzy-answers (get-input-sent-node) #:do-microplanning #f)))
            (if (null? fuz-ans)
                (State fuzzy-answers no-result)

                ; Fuzzy matcher may return more than one answers that have the
                ; same score, randomly pick one of them if so
                ; TODO: Should also return other results accordingly, not always the top ones only
                (let* ((ans (list (list-ref fuz-ans (random (length fuz-ans)))))
                       (ans-in-words (List (map Word (string-split (car ans) #\ )))))
                    (State fuzzy-answers ans-in-words)

                    ; TODO: Create a new psi-rule for this QA in the OpenCog AIML format
                )
            )
            (State fuzzy-qa search-finished)
        )
    )
)

(define (do-fuzzy-match)
    (State fuzzy-match search-started)

    (begin-thread
        (let ((fuzzy-results (fuzzy-match-sent (get-input-sent-node) '()))
              (rtn '()))
            ; No result if it's an empty ListLink
            (if (equal? (cog-arity fuzzy-results) 0)
                (State fuzzy-replies no-result)
                (begin
                    (set! rtn (pick-and-generate (cog-outgoing-set fuzzy-results)))
                    (cog-extract fuzzy-results)
                    (if (null? rtn)
                        ; Could happen if none of them can be used to generate
                        ; an actual sentence
                        (State fuzzy-replies no-result)
                        (State fuzzy-replies (List (map Word (string-split rtn #\ ))))
                    )
                )
            )
            (State fuzzy-match search-finished)
        )
    )
)

(define (do-aiml-search)
    (State aiml-search search-started)

    (begin-thread
        (let ((aiml-resp (aiml-get-response-wl (get-input-word-list))))
            ; No result if it's a ListLink with arity 0
            (if (equal? (cog-arity aiml-resp) 0)
                (State aiml-replies no-result)
                (begin
                    (State aiml-replies aiml-resp)

                    ; Update the TV of the psi-rule that will actually execute
                    ; the "Reply" action
                    (cog-set-tv! (car (cog-chase-link 'MemberLink 'ImplicationLink aiml-reply-rule))
                        (cog-tv (aiml-get-selected-rule)))
                )
            )
            (State aiml-search search-finished)
        )
    )
)

(define (say . words)
    (define utterance (string-join (map cog-name words)))

    ; Remove those '[', ']' and '\' that may exist in the output
    (set! utterance (string-filter
        (lambda (c) (not (or (char=? #\[ c) (char=? #\] c) (char=? #\\ c)))) utterance))

    (display utterance)

    ; For sending out the chatbot response via the grounded predicate defined
    ; in ros-behavior-scripting
    (catch #t
        (lambda ()
            (cog-evaluate! (Evaluation (GroundedPredicate "py: say_text") (List (Node utterance))))
        )
        (lambda (key . parameters)
            ; (display "\n(Warning: Failed to call \"py: say_text\" to send out the message.)\n")
            *unspecified*
        )
    )

    (reset-all-states)
)

(define (reply anchor)
    (let ((ans-in-words (cog-chase-link 'StateLink 'ListLink anchor)))
        (if (null? ans-in-words)
            '()
            (apply say (cog-outgoing-set (car ans-in-words)))
        )
    )
)
