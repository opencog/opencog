;; Attempt to reproduce bug

(load "utilities.scm")

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "pln-rb.scm"))

(define (ppc-reload)
  (clear)
  (load "ppc-kb.scm")
  (load "ppc-rb.scm"))

;; Set the random seed of the experiment
(cog-randgen-set-seed! 0)

;; Set loggers levels
(cog-logger-set-level! (cog-ure-logger) "debug")
(cog-logger-set-level! icl-logger "debug")
(cog-logger-set-level! "debug")

;; Set loggers stdout
(cog-logger-set-stdout! icl-logger #t)

;; ;; Set loggers sync (for debugging)
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-sync! icl-logger #t)
;; (cog-logger-set-sync! (cog-ure-logger) #t)

;; Set parameters
(define pss 100)                         ; Problem set size
(define niter 1)                         ; Number of iterations

;; AtomSpace containing the targets in there to no forget them
(define targets-as (cog-new-atomspace))

;; AtomSpace containing the inference traces of a particular run
(define trace-as (cog-new-atomspace))

(define (run-bug)
  (let* ((default-as (cog-set-atomspace! targets-as)) ; Switch to targets-as
         (targets (gen-random-targets pss))) ; Generate targets

    ;; Switch back to the default atomspace
    (cog-set-atomspace! default-as)

    ;; Run all iterations
    (run-iteration targets)))

;; Run iteration i over the given targets and return the list of
;; solved problems.
(define (run-iteration targets)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-bc-mk-corpus (lambda (j)
                             (let* (;; Target
                                    (trg (list-ref targets j))
                                    ;; Run the BC with control-as
                                    ;; while putting the trace in
                                    ;; trace-as
                                    (bc-result (run-bc trg j)))
                               ;; Post-process trace-as and copy the
                               ;; relevant knowledge in history-as
                               (postprocess-corpus)
                               bc-result)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    ;; Return results for each problem
    results))

;; Post-process the trace trace-as by inferring knowledge about
;; preproof, and add all relevant knowledge to the inference history
;; history-as from it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus)
  ;; Reload the postprocessing knowledge and rules
  (ppc-reload)
  (let ((default-as (cog-set-atomspace! trace-as)))
    ;; Copy trace-as to the default atomspace
    (cog-cp-all default-as)
    ;; Switch to the default atomspace
    (cog-set-atomspace! default-as))
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
    results))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, trace-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target j)
  (icl-logger-info "Run BC (j=~a/~a) with target:\n~a"
                   niter (+ j 1) pss target)
  (clear-as trace-as)
  (icl-logger-debug "Right before bugging")
  (clear)
  (reload)
  (let* ((result (pln-bc target))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

