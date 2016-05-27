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

(define-public (is-canned-rule? rule)
    (let ((m (cog-chase-link 'MemberLink 'ConceptNode rule)))
        (if (and (not (null? m)) (equal? (car m) canned-rule))
            #t
            #f
        )
    )
)
