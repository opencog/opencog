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

(define (get-input-word-list)
    (get-input 'ListLink)
)

(define (get-input-sent-node)
    (get-input 'SentenceNode)
)

(define (get-input-text-node)
    (get-input 'Node)
)

(define (is-utterance-type? speechact)
    (Satisfaction (And
        (State input-utterance (Reference (Variable "$l") (Variable "$x") (Variable "$a")))
        (Parse (Variable "$parse") (Variable "$x"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") speechact)
    ))
)

(define (search-started? anchor)
    (Equal (Set search-started) (Get (State anchor (Variable "$s"))))
)

(define (any-result? anchor)
    (Not (Equal (Set no-result) (Get (State anchor (Variable "$f")))))
)

(define (add-thread t)
    (set! all-threads (append all-threads (list t)))
)

(define (cancel-all-threads)
    (map cancel-thread all-threads)
    (set! all-threads '())
)

(define (reset-all-states)
    (State input-utterance no-input-utterance)
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-replies default-state)
    (State fuzzy-match default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa-search default-state)
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
