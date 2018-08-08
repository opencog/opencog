(use-modules (srfi srfi-1)
             (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; word is not define-public, so we have to load terms.scm to test it.
(load "../../../opencog/nlp/chatlang/terms.scm")

(define word (word "mint"))

; Just want to see if the ReferenceLink is there...
(define test-word-result
    (any (lambda (x)
        (and (eq? (cog-type x) 'ReferenceLink)
             (equal? (gdr x) (WordNode "mint"))))
        (cdr word)))
