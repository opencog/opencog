(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(load "test-atomese.scm")

(define sent (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "Richard eats delicious fishes"))))

(define test-result-1
    (if (and (equal? (does-not-contain sent (List (map Word (list "blah" "yeah"))))
                     (stv 1 1))
             (equal? (does-not-contain sent (List (map Word (list "eats" "bar"))))
                     (stv 0 1)))
        #t
        #f))

(define test-result-2
    (if (and (equal? (does-not-start-with sent (List (map Word (list "blah" "yeah"))))
                     (stv 1 1))
             (equal? (does-not-start-with sent (List (map Word (list "Richard" "bar"))))
                     (stv 0 1)))
        #t
        #f))

(define test-result-3
    (if (and (equal? (does-not-end-with sent (List (map Word (list "blah" "yeah"))))
                     (stv 1 1))
             (equal? (does-not-end-with sent (List (map Word (list "fishes" "bar"))))
                     (stv 0 1)))
        #t
        #f))

(define test-result-4
    (if (and (equal? (no-words-in-between sent (Word "eats") (Word "fishes")
                         (List (map Word (list "blah" "yeah"))))
                     (stv 1 1))
             (equal? (no-words-in-between sent (Word "eats") (Word "fishes")
                         (List (map Word (list "delicious" "bar"))))
                     (stv 0 1)))
        #t
        #f))
