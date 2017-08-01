(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

; For getting the lemma
(load "../../../opencog/nlp/chatlang/translator.scm")
(set! test-get-lemma #t)

(define exist-concept (create-concept "foo" "swallow"))
(define concept
    (create-concept "eat" "eat" "ingests" "binge and purge" "~foo"))

(define expected-result
    (list (ReferenceLink
              (LemmaNode "eat")
              (ConceptNode "eat"))
          (ReferenceLink
              (WordNode "ingests")
              (ConceptNode "eat"))
          (ReferenceLink
              (PhraseNode "binge and purge")
              (ConceptNode "eat"))
          (ReferenceLink
              (ConceptNode "foo")
              (ConceptNode "eat"))))

(define test-ui-concept-result
    (equal? concept expected-result))
