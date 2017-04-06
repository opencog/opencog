(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define lemma (lemma "is"))

; Just want to see if the LemmaLink is there...
(define test-lemma-result
    (not (null? (filter (lambda (x)
        (and (eq? (cog-type x) 'LemmaLink)
             (equal? (gdr x) (WordNode "is"))))
        (cdr lemma)))))
