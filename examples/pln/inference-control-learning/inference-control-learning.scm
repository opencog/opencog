;; Contain the main inference control learning experiment loop

(load "utilities.scm")

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "rb.scm"))

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
(cog-logger-set-level! (cog-ure-logger) "debug")
(cog-logger-set-level! icl-logger "debug")
(cog-logger-set-level! "debug")

;; Set loggers stdout
(cog-logger-set-stdout! icl-logger #t)

;; ;; Set loggers sync (for debugging)
(cog-logger-set-sync! #t)
(cog-logger-set-sync! icl-logger #t)
(cog-logger-set-sync! (cog-ure-logger) #t)

;; Set parameters
(define pss 5)                          ; Problem set size
(define niter 1)                         ; Number of iterations

(define (run-experiment)
  (icl-logger-info "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (history-as (cog-new-atomspace))   ; Initialize the Global
                                            ; Inference History
         (control-as (cog-new-atomspace)))  ; Initialize Inference
                                            ; Control Rules
    ;; Function for running all iterations given the iteration index,
    ;; i. Return the list of solved problems for each iteration (list
    ;; of list).
    (define (run-iterations-rec i)
      (if (< i niter)
          (let* ((sol (run-iteration targets history-as control-as i)))
            (cons sol (run-iterations-rec (+ i 1))))
          '()))
    ;; Run all iterations
    (run-iterations-rec 0)))

;; Run iteration i over the given targets and return the list of
;; solved problems.
(define (run-iteration targets history-as control-as i)
  (icl-logger-info "Run iteration (i=~a/~a)" (+ i 1) niter)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-bc-mk-corpus (lambda (j)
                             (let* (;; AtomSpace where to the record
                                    ;; the inference traces for that run
                                    (trace-as (cog-new-atomspace))
                                    ;; Target
                                    (trg (list-ref targets j))
                                    ;; Run the BC with control-as
                                    ;; while putting the trace in
                                    ;; trace-as
                                    (bc-result (run-bc trg
                                                       trace-as
                                                       control-as
                                                       i j)))
                               ;; Post-process trace-as and copy the
                               ;; relevant knowledge in history-as
                               (postprocess-corpus trace-as history-as)
                               bc-result)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    (icl-logger-info "Number of solved problems = ~a" sol_count)

    ;; Build inference control rules for the next iteration
    (mk-ic-rules history-as control-as)
    ;; Return results for each problem
    results))

;; Post-process the trace trace-as by inferring knowledge about
;; preproof, and add all relevant knowledge to the inference history
;; history-as from it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus trace-as history-as)
  (icl-logger-info "Post-process trace, add to inference history")
  ;; Reload the postprocessing knowledge and rules
  (ppc-reload)
  (let ((default-as (cog-set-atomspace! trace-as)))
    ;; Copy trace-as to the default atomspace
    (cog-cp-all default-as)
    ;; Switch to the default atomspace
    (cog-set-atomspace! default-as)
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
      (icl-logger-debug "Results:\n~a" results)
      (cog-cp (cog-outgoing-set results) history-as)
      (icl-logger-debug "history-as after copying results")
      (icl-logger-debug-atomspace history-as)
      (cog-cp (cog-get-atoms 'ExecutionLink) history-as)
      (icl-logger-debug "history-as after copying execution links")
      (icl-logger-debug-atomspace history-as)
      (remove-dangling-atoms history-as)
      (icl-logger-debug "history-as after removing dangling atoms")
      (icl-logger-debug-atomspace history-as))))

(define (mk-ic-rules history-as control-as)
  (icl-logger-debug "history-as before anything")
  (icl-logger-debug-atomspace history-as)
  (clear)
  (icl-logger-debug "Default atomspace before copying history-as to it")
  (icl-logger-debug-atomspace (cog-atomspace))
  (cp-as history-as (cog-atomspace))
  (icl-logger-debug "Default atomspace after copying history-as to it")
  (icl-logger-debug-atomspace (cog-atomspace))
  )

  ;; (icl-logger-debug "history-as before mk-ic-rules")
  ;; (icl-logger-debug-atomspace history-as)
  ;; (icl-logger-info "Build inference control rules from the inference history")
  ;; ;; Reload the rule base for producing inference control rules
  ;; (icr-reload)
  ;; (icl-logger-debug "Default atomspace after loading the icr rule base:")
  ;; (icl-logger-debug-atomspace (cog-atomspace))
  ;; (let ((default-as (cog-set-atomspace! history-as)))
  ;;   (icl-logger-debug "Default atomspace switching to history-as")
  ;;   (icl-logger-debug-atomspace (cog-atomspace))
  ;;   ;; Copy history-as to the default atomspace
  ;;   (cog-cp-all default-as)
  ;;   (icl-logger-debug "Default atomspace after adding the inference history:")
  ;;   (icl-logger-debug-atomspace default-as)
  ;;   ;; Switch to the default atomspace
  ;;   (cog-set-atomspace! default-as)
  ;;   (icl-logger-debug "Default atomspace after switching to default atomspace:")
  ;;   (icl-logger-debug-atomspace (cog-atomspace))
  ;;   ;; Define BC target and vardecl
  ;;   (let* ((vardecl (TypedVariable
  ;;                      (Variable "$Rule")
  ;;                      (Type "BindLink")))
  ;;          (target (ImplicationScope
  ;;                    (VariableList
  ;;                      (Variable "$T")
  ;;                      (TypedVariable
  ;;                        (Variable "$A")
  ;;                        (Type "BindLink"))
  ;;                      (Variable "$L")
  ;;                      (TypedVariable
  ;;                        (Variable "$B")
  ;;                        (Type "BindLink")))
  ;;                    (Execution
  ;;                      (Schema "URE:BC:expand-and-BIT")
  ;;                      (List
  ;;                        (DontExec (Variable "$A"))
  ;;                        (Variable "$L")
  ;;                        (DontExec (Variable "$Rule")))
  ;;                      (DontExec (Variable "$B")))
  ;;                    (Evaluation
  ;;                      (Predicate "URE:BC:preproof")
  ;;                      (List
  ;;                        (DontExec (Variable "$A"))
  ;;                        (Variable "$T")))))
  ;;          (results (icr-bc target #:vardecl vardecl)))
  ;;     (icl-logger-debug "Query target = ~a" target)
  ;;     ;; Copy inference control rules to the Inference Control Rules
  ;;     ;; atomspace.
  ;;     (icl-logger-debug "Results:\n~a" results)
  ;;     (cog-cp (cog-outgoing-set results) control-as))))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, trace-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target trace-as control-as i j)
  (icl-logger-info "Run BC (i=~a/~a,j=~a/~a) with target:\n~a"
                   (+ i 1) niter (+ j 1) pss target)
  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as)

  (reload)
  (let* ((result (pln-bc target #:trace-as trace-as #:control-as control-as))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

