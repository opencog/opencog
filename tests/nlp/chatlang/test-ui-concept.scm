(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

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
              (ListLink
                  (WordNode "binge")
                  (WordNode "and")
                  (WordNode "purge"))
              (ConceptNode "eat"))
          (ReferenceLink
              (ConceptNode "foo")
              (ConceptNode "eat"))))

(define test-ui-concept-result
    (equal? concept expected-result))
