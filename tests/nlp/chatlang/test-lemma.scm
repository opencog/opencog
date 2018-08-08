(use-modules (srfi srfi-1)
             (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; lemma is not define-public, so we have to load terms.scm to test it.
(load "../../../opencog/nlp/chatlang/translator.scm")
(load "../../../opencog/nlp/chatlang/terms.scm")
(set! test-get-lemma #t)

(define ilemma (lemma "is"))

; Just want to see if the LemmaLink is there...
(define test-lemma-result
    (any (lambda (x)
        (and (eq? (cog-type x) 'LemmaLink)
             (equal? (gdr x) (WordNode "be"))))
        (cdr ilemma)))
