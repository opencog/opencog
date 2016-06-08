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
