(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang))

(define concept
    (chat-concept "eat" (list "eat" "ingest" "binge and purge")))

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
              (ConceptNode "eat"))))

(define test-ui-concept-result
    (equal? concept expected-result))
