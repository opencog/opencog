(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define lemma (lemma "is"))

; Just want to see if the LemmaLink is there...
(define test-result
    (and (eq? (cog-type (cadr lemma)) 'LemmaLink)
         (equal? (gdr (cadr lemma)) (WordNode "is"))))
