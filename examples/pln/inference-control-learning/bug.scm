(load "bug-utilities.scm")

;; AtomSpace containing the inference traces of a particular run
(define trace-as (cog-new-atomspace))

;; Run and-bit-prior rule base over trace-as and copy its results to
;; history-as.
(define (run-bug)
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as)) ;; <--- bug
         ;; Load and-bit-prior rule base
         (dummy (load "bug-and-bit-prior.scm"))
         ;; Define BC target and vardecl
         (target (Evaluation
                   (Predicate "URE:BC:preproof")
                   (List
                     (Variable "$A")
                     (Variable "$T"))))
         (vardecl (VariableList
                    (TypedVariable
                      (Variable "$A")
                      (Type "DontExecLink"))
                    (Variable "$T")))
         ;; Run and-bit-prior-rule over trace-as
         (results (cog-bind and-bit-prior-rule)))  ;; <--- bug
    (cog-logger-debug "apply-and-bit-prior trace atomspace:")
    (cog-logger-debug-atomspace trace-as)
    (cog-logger-debug "apply-and-bit-prior default atomspace:")
    (cog-logger-debug-atomspace (cog-atomspace))))

(run-bug)
(quit)
