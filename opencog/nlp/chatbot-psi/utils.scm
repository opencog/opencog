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
