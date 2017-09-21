;; Contain the main inference control learning experiment loop

(load "utilities.scm")

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "pln-rb.scm"))

(define (icr-reload)
  (clear)
  (load "icr-rb.scm"))

;; Set the random seed of the experiment
(cog-randgen-set-seed! 0)

;; Set loggers levels
(cog-logger-set-level! "debug")
(cog-logger-set-level! icl-logger "debug")
(cog-logger-set-level! (cog-ure-logger) "debug")

;; Set loggers stdout
(cog-logger-set-stdout! #t)
(cog-logger-set-stdout! icl-logger #t)
;; (cog-logger-set-stdout! (cog-ure-logger) #t)

;; Set loggers sync (for debugging)
(cog-logger-set-sync! #t)
(cog-logger-set-sync! icl-logger #t)
(cog-logger-set-sync! (cog-ure-logger) #t)

;; Set parameters
(define pss 1)                    ; Problem set size
(define niter 1)                    ; Number of iterations
(define piter 3)                   ; Number of iterations used for each problem

;; AtomSpace containing the targets in there to no forget them
(define targets-as (cog-new-atomspace))

;; AtomSpace containing the inference traces of a particular run
(define trace-as (cog-new-atomspace))

(define (run-bug)
  (icl-logger-info "Start experiment")
  (let* ((default-as (cog-set-atomspace! targets-as)) ; Switch to targets-as
         (targets (gen-random-targets pss))) ; Generate targets

    ;; Switch back to the default atomspace
    (cog-set-atomspace! default-as)

    (run-iteration targets 0)))

;; Run iteration i over the given targets and return the list of
;; solved problems.
(define (run-iteration targets i)
  (icl-logger-info "Run iteration (i=~a/~a)" (+ i 1) niter)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-bc-mk-corpus (lambda (j)
                             (let* (;; Target
                                    (trg (list-ref targets j))
                                    ;; Run the BC with control-as
                                    ;; while putting the trace in
                                    ;; trace-as
                                    (bc-result (run-bc trg i j)))
                               ;; Post-process trace-as and copy the
                               ;; relevant knowledge in history-as
                               (postprocess-corpus)
                               bc-result)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    (icl-logger-info "Number of solved problems = ~a" sol_count)

    ;; Return results for each problem
    results))

;; Post-process the trace trace-as by inferring knowledge about
;; preproof, and add all relevant knowledge to the inference history
;; history-as from it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus)
  (icl-logger-info "Post-process trace, add to inference history")

  (icl-logger-debug "Trace AtomSpace 1:")
  (icl-logger-debug-atomspace trace-as)

  ;; Apply preprocessing rules (or rule bases) to trace-as (the order
  ;; is important)
  (apply-proof-is-preproof)
  (icl-logger-debug "Trace AtomSpace 2:")
  (icl-logger-debug-atomspace trace-as)
  (apply-preproof-expander-is-preproof)
  (icl-logger-debug "Trace AtomSpace 3:")
  (icl-logger-debug-atomspace trace-as)
  (apply-and-bit-prior)
  (icl-logger-debug "Trace AtomSpace 4:")
  (icl-logger-debug-atomspace trace-as)
)

;; Turn apply proof-is-preproof rule to trace-as and copy the results
;; to history-as
(define (apply-proof-is-preproof)
  (load "proof-is-preproof.scm")
  (let* ((default-as (cog-set-atomspace! trace-as))
         (results (cog-bind proof-is-preproof-rule)))
    (remove-dangling-atoms trace-as)
    (cog-set-atomspace! default-as)))

;; Run preproof-expander-is-preproof rule base over trace-as and copy
;; the results to history-as
(define (apply-preproof-expander-is-preproof)
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as))
         ;; Load preproof-expanded-is-preproof rule base
         (dummy (load "preproof-expander-is-preproof.scm"))
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
         (results (pep-bc target #:vardecl vardecl)))
    ;; Copy post-processed inference traces to the inference
    ;; history.
    (icl-logger-debug "apply-preproof-example-is-preproof results = ~a" results)
    (cog-delete results)
    (remove-dangling-atoms trace-as)
    (cog-set-atomspace! default-as)))

;; Run and-bit-prior rule base over trace-as and copy its results to
;; history-as.
(define (apply-and-bit-prior)
  (icl-logger-debug "apply-and-bit-prior Trace AtomSpace 1:")
  (icl-logger-debug-atomspace trace-as)
  
  (let* (;; Switch to trace-as
         (default-as (cog-set-atomspace! trace-as))
         ;; Load and-bit-prior rule base
         (dummy (load "and-bit-prior.scm"))
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
         (results (abp-bc target #:vardecl vardecl)))
    (icl-logger-debug "apply-and-bit-prior Trace AtomSpace 2:")
    (icl-logger-debug-atomspace trace-as)
    ;; Copy post-processed inference traces to the inference
    ;; history.
    (icl-logger-debug "apply-and-bit-prior Trace AtomSpace 3:")
    (icl-logger-debug-atomspace trace-as)
    (icl-logger-debug "apply-and-bit-prior results = ~a" results)
    (cog-delete results)
    (icl-logger-debug "apply-and-bit-prior Trace AtomSpace 4:")
    (icl-logger-debug-atomspace trace-as)
    (remove-dangling-atoms trace-as)
    (icl-logger-debug "apply-and-bit-prior Trace AtomSpace 5:")
    (icl-logger-debug-atomspace trace-as)
    (cog-set-atomspace! default-as)))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, trace-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target i j)
  (icl-logger-info "Run BC (i=~a/~a,j=~a/~a) with target:\n~a"
                   (+ i 1) niter (+ j 1) pss target)
  (clear-as trace-as)

  (reload)

  (let* ((result (pln-bc target #:trace-as trace-as))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

(run-bug)
