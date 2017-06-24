(use-modules (srfi srfi-1)
             (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; phrase is not define-public, so we have to load terms.scm to test it.
(load "../../../opencog/nlp/chatlang/translator.scm")
(load "../../../opencog/nlp/chatlang/terms.scm")

(define phrase (phrase "John Smith"))

; Just want to see if the ReferenceLink is there...
(define test-phrase-result
    (equal? 2 (length
        (filter (lambda (x)
            (and (eq? (cog-type x) 'ReferenceLink)
                 (or (equal? (gdr x) (WordNode "John"))
                     (equal? (gdr x) (WordNode "Smith")))))
        (cdr phrase)))))
