;; Contain the main inference control learning experiment loop

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
;; (cog-logger-set-stdout! #t)
(cog-logger-set-stdout! icl-logger #t)

;; ;; Set loggers sync (for debugging)
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-sync! icl-logger #t)
;; (cog-logger-set-sync! (cog-ure-logger) #t)

;; Set parameters
(define pss 10)                          ; Problem set size
(define niter 2)                         ; Number of iterations

;; AtomSpace containing the targets in there to no forget them
(define targets-as (cog-new-atomspace))

;; AtomSpace containing the inference traces of a particular run
(define trace-as (cog-new-atomspace))

;; AtomSpace containing the global inference history
(define history-as (cog-new-atomspace))

;; AtomSpace containing the control rules
(define control-as (cog-new-atomspace))

(define (run-experiment)
  (icl-logger-info "Start experiment")
  (let* ((default-as (cog-set-atomspace! targets-as)) ; Switch to targets-as
         (targets (gen-random-targets pss))) ; Generate targets

    ;; Function for running all iterations given the iteration index,
    ;; i. Return the list of solved problems for each iteration (list
    ;; of list).
    (define (run-iterations-rec i)
      (if (< i niter)
          (let* ((sol (run-iteration targets i)))
            (cons sol (run-iterations-rec (+ i 1))))
          '()))

    ;; Switch back to the default atomspace
    (cog-set-atomspace! default-as)

    ;; Run all iterations
    (run-iterations-rec 0)))

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

    ;; Build inference control rules for the next iteration
    (mk-ic-rules)
    ;; Return results for each problem
    results))

;; Post-process the trace trace-as by inferring knowledge about
;; preproof, and add all relevant knowledge to the inference history
;; history-as from it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus)
  (icl-logger-info "Post-process trace, add to inference history")
  ;; Reload the postprocessing knowledge and rules
  (ppc-reload)
  ;; Copy trace-as to the default atomspace
  (cp-as trace-as (cog-atomspace))
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
    ;; Copy post-processed inference traces to the inference
    ;; history. Execution relationships + preproof evaluations
    (cog-cp (cog-outgoing-set results) history-as)
    (cog-cp (cog-get-atoms 'ExecutionLink) history-as)
    (remove-dangling-atoms history-as)))

(define (mk-ic-rules)
  (icl-logger-info "Build inference control rules from the inference history")

  ;; Reload the rule base for producing inference control rules
  (icr-reload)

  ;; Copy history-as to the default atomspace
  (cp-as history-as (cog-atomspace))

  ;; Load PLN to have access to the PLN rules
  (load "pln-rb.scm")

  ;; Define BC target and vardecl
  (let* ((vardecl (TypedVariable
                    (Variable "$Rule")
                    (Type "DefinedSchemaNode")))
         (impl-vardecl (VariableList
                         (Variable "$T")
                         (TypedVariable
                           (Variable "$A")
                           (Type "DontExecLink"))
                         (Variable "$L")
                         (TypedVariable
                           (Variable "$B")
                           (Type "DontExecLink"))))
         (impl-antecedant (And
                            (Evaluation
                              (Predicate "URE:BC:preproof")
                              (List
                                (Variable "$A")
                                (Variable "$T")))
                            (Execution
                              (Schema "URE:BC:expand-and-BIT")
                              (List
                                (Variable "$A")
                                (Variable "$L")
                                (DontExec (Variable "$Rule")))
                              (Variable "$B"))))
         (impl-consequent (Evaluation
                            (Predicate "URE:BC:preproof")
                            (List
                              (Variable "$B")
                              (Variable "$T"))))
         (target (ImplicationScope
                   impl-vardecl
                   impl-antecedant
                   impl-consequent))

         ;; Instantiate the targets as required by icr-bc
         (rules-to-targets (Bind
                             vardecl
                             (Member
                               (Variable "$Rule")
                               pln-rbs)
                             target))
         (targets (cog-bind rules-to-targets))

         ;; Evaluate the antecedants
         (antecedants-result (pp-icr-bc impl-antecedant))

         ;; Produce the inference control rules
         (results (icr-bc target #:vardecl vardecl)))
    ;; Copy inference control rules to the Inference Control Rules
    ;; atomspace.
    (cog-cp (cog-outgoing-set results) control-as))

  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, trace-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target i j)
  (icl-logger-info "Run BC (i=~a/~a,j=~a/~a) with target:\n~a"
                   (+ i 1) niter (+ j 1) pss target)
  (clear-as trace-as)

  (reload)

  (let* ((result (pln-bc target #:trace-as trace-as #:control-as control-as))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

(run-experiment)
