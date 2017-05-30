(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define exist-concept (create-concept "foo" "swallow"))
(define concept
    (create-concept "eat" "eat" "ingest" "binge and purge" "~foo"))

(define expected-result
    (list (ReferenceLink
              (WordNode "eat")
              (ConceptNode "eat"))
          (ReferenceLink
              (WordNode "ingest")
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
