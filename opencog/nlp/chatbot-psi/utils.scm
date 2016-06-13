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

(define (reset-all-states)
    (State input-utterance no-input-utterance)
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa-search default-state)
)
