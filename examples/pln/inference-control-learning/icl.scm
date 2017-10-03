;; Contain the main inference control learning experiment loop

(load "icl-utilities.scm")

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
(cog-logger-set-level! "info")
(cog-logger-set-level! icl-logger "debug")
(cog-logger-set-level! (cog-ure-logger) "info")

;; Set loggers stdout
;; (cog-logger-set-stdout! #t)
(cog-logger-set-stdout! icl-logger #t)
;; (cog-logger-set-stdout! (cog-ure-logger) #t)

;; ;; Set loggers sync (for debugging)
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-sync! icl-logger #t)
;; (cog-logger-set-sync! (cog-ure-logger) #t)

;; Set parameters
(define pss 100)                    ; Problem set size
(define niter 2)                    ; Number of iterations
(define piter 30)                   ; Number of iterations used for each problem

;; AtomSpace containing the targets in there to no forget them
(define targets-as (cog-new-atomspace))

;; AtomSpace containing the control rules
(define control-as (cog-new-atomspace))

;; AtomSpace containing the history across problems and iterations
(define history-as (cog-new-atomspace))

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
         (run (lambda (j)
                (let* (;; Target
                       (trg (list-ref targets j))
                       ;; Run the BC, return a pair with produced
                       ;; history and result (true or false)
                       (result (run-bc-mk-history trg i j)))
                  result)))
         (histories-results (map run (iota pss)))
         (histories (map car histories-results))
         (results (map cdr histories-results))
         (sol_count (count values results)))

    (icl-logger-info "Number of solved problems = ~a" sol_count)

    ;; Copy all atomspaces for histories to history-as
    (icl-logger-info "Move all problem histories to history-as")
    (union-as history-as histories)

    ;; Return results for each problem
    results))

;; Run the backward chainer on the given target, postprocess its trace
;; and fill an history atomspace with it. Return a pair with history
;; as first element and the result of running the BC (true or false).
(define (run-bc-mk-history target i j)
  (let* ((trace-as (cog-new-atomspace))
         (result (run-bc target i j trace-as))
         (history-as (mk-history trace-as)))
    (cons history-as result)))

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

    history-as))

;; Apply proof-is-preproof rule to trace-as and copy the results to
;; history-as
(define (apply-proof-is-preproof trace-as history-as)
  (let* ((default-as (cog-set-atomspace! trace-as))
         (dummy (load "proof-is-preproof.scm"))
         (results (cog-bind proof-is-preproof-rule)))
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
         (target (Evaluation
                   (Predicate "URE:BC:preproof-of")
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

(define (mk-control-rules)
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
                              (Predicate "URE:BC:preproof-of")
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
                            (Predicate "URE:BC:preproof-of")
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
    (icl-cp control-as (cog-outgoing-set results)))

  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, trace-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target i j trace-as)
  (icl-logger-info "Run BC (i=~a/~a,j=~a/~a) with target:\n~a"
                   (+ i 1) niter (+ j 1) pss target)

  (reload)

  (let* ((result (pln-bc target #:trace-as trace-as #:control-as control-as))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

(run-experiment)
