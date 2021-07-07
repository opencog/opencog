; --------------------------------------------------
; Setup for the test
(use-modules (opencog test-runner))

(opencog-test-runner)

(define ghost-utest "GhostUTest")

; --------------------------------------------------
; Setup for GHOST
(use-modules
  (opencog)
  (opencog nlp)
  (opencog nlp oc)
  (opencog nlp relex2logic)
  (opencog nlp chatbot)
  (opencog openpsi)
  (opencog ghost)
  (opencog ghost procedures))

; Set the address for relex server
(set-relex-server-host)

; Helper for this test
(define* (get-result #:optional input)
  ; Send the input to GHOST
  (if input (ghost input))
  ; Should be able to trigger the rule and get the result
  ; almost instantly, but let's give it 2 seconds just
  ; to be safe
  (sleep 2)
  (if (nil? (ghost-get-result))
    (string)
    (string-join (map cog-name (ghost-get-result)))))

; --------------------------------------------------
; Test
(test-begin ghost-utest)
(define ghost-run-test "GHOST run")

; Not testing with ECAN, for now
(ghost-set-sti-weight 0)
(ghost-af-only #f)

(define pred-true #f)
(define-public (some_pred) (if pred-true (stv 1 1) (stv 0 1)))
(define-public (some_schema) (List (Word "and") (Word "schema")))

; Parse a few rules and test the basic flow
(ghost-parse "
  goal: (utest-goal=1)
  r: (test reactive rule) done reactive
    j1: (test rejoinder) done rejoinder

  p: (test proactive rule) done proactive

  r: RR (^some_pred()) done predicate ^some_schema()
")

; The loop count should be zero before starting the loop
(test-assert ghost-run-test (= 0 (psi-loop-count (ghost-get-component))))

(ghost-run)
(sleep 1)

; The loop count should be greater than zero after starting the loop
(test-assert ghost-run-test (< 0 (psi-loop-count (ghost-get-component))))

(test-equal ghost-run-test "done reactive" (get-result "test reactive rule"))
(test-equal ghost-run-test "done rejoinder" (get-result "test rejoinder"))
(test-equal ghost-run-test "done proactive" (get-result "test proactive rule"))
(set! pred-true #t)
(test-equal ghost-run-test "done predicate and schema" (get-result))

(ghost-halt)

; The loop count should remains unchanged after halting the loop
(define final-cnt (psi-loop-count (ghost-get-component)))
(sleep 1)
(test-assert ghost-run-test (= final-cnt (psi-loop-count (ghost-get-component))))

; End of the test
(test-end ghost-utest)
