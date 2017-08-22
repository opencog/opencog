;; Attempt to reproduce bug

(use-modules (srfi srfi-1))

(define (ppc-reload)
  (clear)
  (load "ppc-kb.scm")
  (load "ppc-rb.scm"))

;; Set parameters
(define pss 100)                         ; Problem set size

;; Run bug
(define (run-bug)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-iteration (lambda (j)
                          (display (format "run-iteration (j=~a/~a)\n"
                                           (+ j 1) pss))

                          (call-ure)))
         (results (map run-iteration (iota pss))))
    ;; Return results for each ure call
    results))

(define (call-ure)
  ;; Reload the postprocessing knowledge and rules
  (ppc-reload)
  ;; Define BC target and vardecl
  (let* ((target (Evaluation
                   (Predicate "URE:BC:preproof")
                   (List
                     (Variable "$A")
                     (Variable "$T"))))
         (vardecl (VariableList
                    (TypedVariable
                      (Variable "$A")
                      (Type "DontExecLink"))
                    (Variable "$T")))
         (results (ppc-bc target #:vardecl vardecl)))
    (display "Right before bugging\n")
    (clear)
    results))
