(use-modules (ice-9 threads))
(use-modules (opencog))
(use-modules (opencog logger))
(use-modules (opencog eva-model))
(use-modules (opencog nlp chatbot-eva))

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
            (if (nil? generated)
                ; Do it again if the chosen one can't be used to generate a sentence
                (pick-and-generate (delete! generated list-of-results))
                (begin
                    ; Get the speech act of the reply
                    (State fuzzy-reply-type
                        (car (filter (lambda (node) (string-suffix? "SpeechAct" (cog-name node)))
                            (cog-filter 'DefinedLinguisticConceptNode (cog-get-all-nodes (gar picked)))))
                    )
                    ; Get the score
                    (State fuzzy-reply-conf (gdr picked))
                    generated
                )
            )
        )
    )
)

;-------------------------------------------------------------------------------

(define (call-chatbot-eva)
    (State chatbot-eva process-started)

    (begin-thread
        (imperative-process (get-input-sent-node))
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (call-fuzzy)
    (State fuzzy process-started)

    (begin-thread
        (let ((fuzzy-results (fuzzy-match-sent (get-input-sent-node) '())))
            ; No result if it's an empty ListLink
            (if (equal? (cog-arity fuzzy-results) 0)
                (State fuzzy-reply no-result)
                (let ((rtn (pick-and-generate (cog-outgoing-set fuzzy-results))))
                    (cog-extract! fuzzy-results)
                    (if (nil? rtn)
                        ; Could happen if none of them can be used to generate
                        ; an actual sentence
                        (State fuzzy-reply no-result)
                        (State fuzzy-reply (List (map Word (string-split rtn #\ ))))
                    )
                )
            )
            (State fuzzy process-finished)
        )
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (call-aiml)
    (State aiml process-started)

    (begin-thread
        (let ((aiml-resp (aiml-get-response-wl (get-input-word-list))))
            ; No result if it's a ListLink with arity 0
            (if (equal? (cog-arity aiml-resp) 0)
                (State aiml-reply no-result)
                (State aiml-reply aiml-resp)
            )
            (State aiml process-finished)
        )
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (call-emotion-state-response)
    (State emotion-state-reply (emotion-state-inquiry-reply))
    (State emotion-state process-finished))

(define (emotion-state-inquiry-reply)
    ; psi-current-emotion-state node below gets defined in ros-behavior-scripting
    ; in psi-dyanmics.scm, but that might not be loaded, so defining it here as
    ; well. Perhaps this should be defined in openpsi/psi-updater.scm and made
    ; part of the openpsi module.
    (define psi-current-emotion-state
        (Concept (string-append psi-prefix-str "current emotion state")))
    ;(define current-emotion (psi-get-value psi-current-emotion-state))
    (define current-emotion (cog-outgoing-set (cog-execute! (Get
        (TypedVariable (Variable "$f") (Type "ConceptNode"))
        (State psi-current-emotion-state (Variable "$f"))))))

    ;(display "in emotion-state-inquiry-response \n")
    (if (not (nil? current-emotion))
        (let ((current-emotion-word (substring (cog-name (car current-emotion))
                (string-length psi-prefix-str))))
            (List (text2wordnodes
                (string-append "I feel " current-emotion-word))))
        (List (text2wordnodes "I am doing okay"))
    )
)

(define (count-and-reply schema)
    (define num (cog-name (cog-execute! schema)))
    (define num-no-dot (string-take num (string-index num #\.)))
    (apply say (text2wordnodes (string-append "I see " num-no-dot)))

    ; Return for the GroundedSchemaNode
    (Set)
)

(define-public (say . words)
    (define utterance "")

    ; This is for getting sentiment info, e.g.
    ; Inheritance
    ;     Concept "happy"
    ;     Concept "Positive"
    (map (lambda (w)
        (set! utterance (string-append utterance " " (cog-name w)))
        (if sentiment-analysis
            (map (lambda (i)
                (if (equal? (cog-type i) 'InheritanceLink)
                    (cond
                        ((equal? (cog-name (gdr i)) "Positive")
                            (set! utterance (string-append utterance
                                "<Positive, " (number->string (cog-mean i)) ">")))
                        ((equal? (cog-name (gdr i)) "Negative")
                            (set! utterance (string-append utterance
                                "<Negative, " (number->string (cog-mean i)) ">")))
                    )))
            (cog-incoming-set (Concept (cog-name w))))))
        words)

    ; Remove those '[', ']' and '\' that may exist in the output
    ; The square brackets are sometimes generated by Link Parser (which
    ; indicates that a grammatical interpretation of the sentence is
    ; found by deleting this word in the sentence using null links)
    ; The backslash is sometimes generated by AIML rules
    ; TODO: Should actually clean up the WordNodes instead
    (set! utterance (string-trim (string-filter
        (lambda (c) (not (or (char=? #\[ c) (char=? #\] c) (char=? #\\ c)))) utterance)))

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

    (reset-all-chatbot-states)

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (reply anchor)
    (let ((ans-in-words (cog-chase-link 'StateLink 'ListLink anchor)))
        (if (nil? ans-in-words)
            ; Return for the GroundedSchemaNode
            (Set)
            (apply say (cog-outgoing-set (car ans-in-words)))
        )
    )
)

;-------------------------------------------------------------------------------

(for-each
    (lambda (w)
        (Evaluation (Predicate "quick reply")
            (List (map Word (string-split w #\ ))))
    )
    (list "ok" "alright" "sure" "keep going" "go ahead" "carry on")
)

(define (ack-the-statement)
    (define qr (cog-execute! (RandomChoice (Get
        (TypedVariable (Glob "$reply") (Type "WordNode"))
            (Evaluation (Predicate "quick reply")
                (List (Glob "$reply")))))))
    (apply say (cog-outgoing-set qr))
)
