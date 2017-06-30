(use-modules (srfi srfi-1) (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog openpsi))

; unordered-matching is not define-public, so we have to load
; terms.scm to test it.
(load "../../../opencog/nlp/chatlang/translator.scm")
(load "../../../opencog/nlp/chatlang/terms.scm")

(define w (cons 'word "drink"))
(define l (cons 'lemma "eat"))
(define p (cons 'phrase "John Smith"))
(define c (cons 'concept "play"))
(define unordered (unordered-matching (list w l p c)))

(define test-unordered-result
    (equal? 5 (length
        (filter (lambda (x)
            (and (or (eq? (cog-type x) 'ReferenceLink)
                     (eq? (cog-type x) 'LemmaLink)
                     (eq? (cog-type x) 'EvaluationLink))
                 (or (equal? (gdr x) (WordNode "drink"))
                     (equal? (gdr x) (WordNode "eat"))
                     (equal? (gdr x) (WordNode "John"))
                     (equal? (gdr x) (WordNode "Smith"))
                     (and (equal? (gar x)
                                  (GroundedPredicateNode "scm: chatlang-concept?"))
                          (equal? (gadr x) (ConceptNode "play"))))))
        (cdr unordered)))))
