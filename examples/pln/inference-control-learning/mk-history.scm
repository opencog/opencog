;; AtomSpace containing the history across problems and iterations
(define history-as (cog-new-atomspace))

;; Post-process the trace trace-as by inferring knowledge about
;; preproof, and add all relevant knowledge to the inference history
;; history-as from it, leaving out cruft like ppc-kb and such. Fill
;; the result in a new atomspace history-as and return it.
(define (mk-history trace-as)
  (let* ((history-as (cog-new-atomspace)))
    (icl-logger-info "Post-process trace, add to inference history")

    ;; Apply preprocessing rules (or rule bases) to trace-as (the order
    ;; is important)
    (apply-proof-is-preproof trace-as history-as)
    (apply-preproof-expander-is-preproof trace-as history-as)
    (apply-and-bit-prior trace-as history-as)

    ;; Copy Execution relationships to history-as to capture the
    ;; expansions and remove cruft from it
    (icl-cp history-as (cog-get-atoms-as trace-as 'ExecutionLink))
    (remove-dangling-atoms history-as)

    ;; (icl-logger-debug "Per Problem History AtomSpace:")
    ;; (icl-logger-debug-atomspace history-as)

    history-as))

;; Apply proof-is-preproof rule to trace-as and copy the results to
;; history-as
(define (apply-proof-is-preproof trace-as history-as)
  (let* ((default-as (cog-set-atomspace! trace-as))
         (dummy (load "proof-is-preproof.scm"))
         (results (cog-execute! proof-is-preproof-rule)))
    (icl-cp history-as (cog-outgoing-set results))
    (remove-dangling-atoms trace-as)
    (extract-hypergraph proof-is-preproof)
    (cog-set-atomspace! default-as)))

;; Run preproof-expander-is-preproof rule base over trace-as and copy
;; the results to history-as
(define (apply-preproof-expander-is-preproof trace-as history-as)
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as))
         ;; Load preproof-expanded-is-preproof rule base
         (dummy (load "preproof-expander-is-preproof.scm"))
         ;; Define BC target and vardecl
         (results (repeat-apply-rule pep-rule (- piter 1))))
    ;; Copy post-processed inference traces to the inference
    ;; history.
    (icl-cp history-as results)
    (remove-dangling-atoms trace-as)
    (cog-set-atomspace! default-as)))

;; Run and-bit-prior rule base over trace-as and copy its results to
;; history-as.
(define (apply-and-bit-prior trace-as history-as)
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as))
         ;; Load and-bit-prior rule base
         (dummy (load "and-bit-prior.scm"))
         ;; Define BC target and vardecl
         (target (preproof-of
                   (List
                     (Variable "$A")
                     (Variable "$T"))))
         (vardecl (VariableList
                    (TypedVariable
                      (Variable "$A")
                      (Type "DontExecLink"))
                    (Variable "$T")))
         ;; Run pep over trace-as
         (results (abp-bc target #:vardecl vardecl)))
    ;; Copy post-processed inference traces to the inference
    ;; history.
    (icl-cp history-as (cog-outgoing-set results))

    (remove-dangling-atoms trace-as)
    (cog-set-atomspace! default-as)))
