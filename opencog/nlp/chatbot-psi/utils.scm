(use-modules (opencog))

;-------------------------------------------------------------------------------
(define (get-word-list sent-node)
    (List
        (append-map
            (lambda (w)
                (if (not (string-prefix? "LEFT-WALL" (cog-name w)))
                    (cog-chase-link 'ReferenceLink 'WordNode w)
                    '()
                )
            )
            (car (sent-get-words-in-order sent-node))
        )
    )
)

(define (get-input type)
    (car (filter
        (lambda (x)
            (if (equal? (cog-type x) type) #t #f))
        (append-map cog-outgoing-set
            (cog-chase-link 'StateLink 'ReferenceLink input-utterance))
    ))
)

(define-public (get-input-word-list)
    (get-input 'ListLink)
)

(define-public (get-input-sent-node)
    (get-input 'SentenceNode)
)

(define-public (get-input-text-node)
    (get-input 'Node)
)

(define-public (get-input-time)
    (cog-name (car (cog-chase-link 'AtTimeLink 'TimeNode (get-input-sent-node))))
)

(define (reset-all-states)
    (State input-utterance no-input-utterance)
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-replies default-state)
    (State fuzzy-match default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa default-state)
    (State duckduckgo-answers default-state)
    (State duckduckgo-search default-state)
    (State wolframalpha-answers default-state)
    (State wolframalpha-search default-state)
    (State chatbot-eva default-state)
)

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
