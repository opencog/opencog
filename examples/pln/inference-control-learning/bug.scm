;; Contain the main inference control learning experiment loop

(load "bug-utilities.scm")

;; Set loggers levels
(cog-logger-set-level! "debug")

;; Set loggers stdout
(cog-logger-set-stdout! #t)

;; Set loggers sync (for debugging)
(cog-logger-set-sync! #t)

;; AtomSpace containing the inference traces of a particular run
(define trace-as (cog-new-atomspace))

;; Run and-bit-prior rule base over trace-as and copy its results to
;; history-as.
(define (run-bug)
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as))
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
         ;; Run pep over trace-as
         (results (abp-bc target #:vardecl vardecl)))  ;; <--- bug
    (cog-logger-debug "apply-and-bit-prior Trace AtomSpace 4:")
    (cog-logger-debug-atomspace trace-as)
    (cog-set-atomspace! default-as)))

(run-bug)
