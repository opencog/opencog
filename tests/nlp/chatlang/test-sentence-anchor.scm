(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi))

(define rule (chat-rule '((lemma "blah")
                          (anchor-start "hi" "there")
                          (anchor-end "bye")
                          (word "wait"))
                        '(say "OK")))

(define term-seq (cog-outgoing-set (gar (car
    (filter (lambda (x) (equal? 'TrueLink (cog-type x)))
            (cog-outgoing-set (gdr (car (psi-get-context rule)))))))))

(define test-start-with-result
    (equal? (list-head term-seq 2)
            (list (Word "hi") (Word "there"))))

(define test-end-with-result
    (equal? (list-tail term-seq 4)
            (list (Word "bye"))))
