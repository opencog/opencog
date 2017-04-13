(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi))

(define rule (chat-rule '((lemma "blah")
                          (unordered-matching "foo" "bar")
                          (word "bleh"))
                        '(say "phew")))

(define term-seq (cog-outgoing-set (gar (car
    (filter (lambda (x) (equal? 'TrueLink (cog-type x)))
            (cog-outgoing-set (gdr (car (psi-get-context rule)))))))))

(define test-unordered-result
    (and (equal? (list-ref term-seq 0)
                 (Word "blah"))
         (equal? (list-ref term-seq 1)
                 (Glob "$unordered"))
         (equal? (list-ref term-seq 2)
                 (Word "bleh"))))
