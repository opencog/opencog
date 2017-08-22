;; Attempt to reproduce bug

(use-modules (srfi srfi-1))

(define (bug-reload)
  (clear)
  (load "bug-kb.scm")
  (load "bug-rb.scm"))

;; Set parameters
(define pss 100)                         ; Problem set size

;; Run bug
(define (run-bug)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-iteration (lambda (i)
                          (display (format "run-iteration (~a/~a)\n"
                                           (+ i 1) pss))
                          (call-ure)))
         (results (map run-iteration (iota pss))))
    ;; Return results for each ure call
    results))

(define (call-ure)
  ;; Reload the postprocessing knowledge and rules
  (bug-reload)
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
         (results (bug-bc target #:vardecl vardecl)))
    (display "Right before bugging\n")
    (clear)
    results))
