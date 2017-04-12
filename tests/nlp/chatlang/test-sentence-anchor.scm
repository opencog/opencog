(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(load "test-atomese.scm")

(define sent (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "Richard eats delicious fishes"))))

(define test-start-with-result
    (if (and (equal? (start-with sent (List (map Word (list "Richard" "eats"))))
                     (stv 1 1))
             (equal? (start-with sent (List (map Word (list "foo" "bar"))))
                     (stv 0 1)))
        #t
        #f))

(define test-end-with-result
    (if (and (equal? (end-with sent (List (map Word (list "fishes"))))
                     (stv 1 1))
             (equal? (end-with sent (List (map Word (list "bar"))))
                     (stv 0 1)))
        #t
        #f))
