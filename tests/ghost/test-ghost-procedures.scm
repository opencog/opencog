(use-modules (opencog test-runner))

(opencog-test-runner)
; Name of test-suite
(define gpt "ghost-procedure-tests")

;------------------------------------------------------------------------------
; Setup

(use-modules
  (opencog)
  (opencog nlp)
  (opencog nlp oc)
  (opencog nlp chatbot)
  (opencog nlp relex2logic)
  (opencog ghost)
  (opencog ghost procedures)
  (srfi srfi-1))

; Set the address for relex server
(set-relex-server-host)

; Utility for testing as sureal part is broken
(define (get-atoms-for-sureal trail)
  (define pln-outputs (cog-value->list
    (cog-value trail (Predicate "inference-results"))))
  (if (nil? pln-outputs)
    '()
    (cog-outgoing-set (filter-for-sureal pln-outputs))))

; For SuReal
(nlp-parse "cats can read")

; Load rule-base for the trail of inference
(load-trail-3)


;------------------------------------------------------------------------------
; Begin test

(test-begin gpt)

; Record current time for getting time-delimited nlp inputs
(pln-record-current-time)

; The inputs
(nlp-parse "reptiles can breathe")
(nlp-parse "lizards are reptiles")

; Do reasoning using the rules from trail-3 rulebase
(do_pln)

; Get results that are filtered to be passed to sureal
(define result (get-atoms-for-sureal rb-trail-3))

; The truth-values in the expected-result are not of concern for the moment
(define expected-result (list
  (EvaluationLink
     (PredicateNode "breathe" (stv 9.7569708e-13 0.0012484395))
     (ListLink (etv 1 0)
        (ConceptNode "lizard")
     )
  ))
)

(test-equal "pln-output" expected-result result)

(test-end gpt)
