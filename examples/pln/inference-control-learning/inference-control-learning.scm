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
(cog-logger-set-level! icl-logger "info")
(cog-logger-set-level! "info")

;; ;; Set loggers sync (for debugging)
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-sync! icl-logger #t)
;; (cog-logger-set-sync! (cog-ure-logger) #t)

;; Set loggers stdout
(cog-logger-set-stdout! icl-logger #t)

;; Set parameters
(define pss 10)                          ; Problem set size
(define niter 3)                         ; Number of iterations

(define (run-experiment)
  (icl-logger-info "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (ih-as (cog-new-atomspace))        ; Initialize the Global
                                            ; Inference History
         (icr-as (cog-new-atomspace)))      ; Initialize Inference
                                            ; Control Rules
    ;; Function for running all iterations given the iteration index,
    ;; i. Return the list of solved problems for each iteration (list
    ;; of list).
    (define (run-iterations-rec i)
      (if (< i niter)
          (let* ((sol (run-iteration targets ih-as icr-as i)))
            (cons sol (run-iterations-rec (+ i 1))))
          '()))
    ;; Run all iterations
    (run-iterations-rec 0)
    (icl-logger-info "Inference History AtomSpace:")
    (icl-logger-info-atomspace ih-as)))

;; Run iteration i over the given targets and return the list of
;; solved problems.
(define (run-iteration targets ih-as icr-as i)
  (icl-logger-info "Run iteration (i=~a/~a)" (+ i 1) niter)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-bc-mk-corpus (lambda (j)
                             (let* (;; AtomSpace where to the record
                                    ;; the inference traces for that run
                                    (tr-as (cog-new-atomspace))
                                    ;; Target
                                    (trg (list-ref targets j))
                                    ;; Run the BC and put the trace in tr-as
                                    (bc-result (run-bc trg tr-as icr-as i j)))
                               ;; Post-process tr-as and copy the
                               ;; relevant knowledge in global-ih-as
                               (postprocess-corpus tr-as ih-as)
                               bc-result)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    (icl-logger-info "Number of problems solved = ~a" sol_count)

    ;; Build inference control rules for the next iteration
    (mk-ic-rules ih-as icr-as)
    ;; Return results for each problem
    results))

;; Post-process the trace tr-as by inferring knowledge about preproof,
;; and add all relevant knowledge to the inference history ih-as from
;; it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus tr-as ih-as)
  (icl-logger-info "Post-process trace, add to inference history")
  ;; Reload the postprocessing knowledge and rules
  (ppc-reload)
  (let ((default-as (cog-set-atomspace! tr-as)))
    ;; Copy tr-as to the default atomspace
    (cog-cp-all default-as)
    ;; Switch to the default atomspace
    (cog-set-atomspace! default-as)
    ;; Define BC target and vardecl
    (let* ((target (Evaluation
                     (Predicate "ICL:preproof")
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
      (cog-cp (cog-outgoing-set results) ih-as)
      (cog-cp (cog-get-atoms 'ExecutionLink) ih-as)
      (remove-dangling-atoms ih-as))))

(define (mk-ic-rules ih-as icr-as)
  (icl-logger-info "Build inference control rules from the inference history")
  ;; Reload the rule base for producing inference control rules
  (icr-reload)
  (let ((default-as (cog-set-atomspace! ih-as)))
    ;; Copy tr-as to the default atomspace
    (cog-cp-all default-as)
    ;; Switch to the default atomspace
    (cog-set-atomspace! default-as)
    ;; Define BC target and vardecl
    (let* ((vardecl (TypedVariable
                       (Variable "$Rule")
                       (Type "BindLink")))
           (target (ImplicationScope
                     (VariableList
                       (Variable "$T")
                       (TypedVariable
                         (Variable "$A")
                         (Type "BindLink"))
                       (Variable "$L")
                       (TypedVariable
                         (Variable "$B")
                         (Type "BindLink")))
                     (Execution
                       (Schema "URE:BC:expand-and-BIT")
                       (List
                         (DontExec (Variable "$A"))
                         (Variable "$L")
                         (DontExec (Variable "$Rule")))
                       (DontExec (Variable "$B")))
                     (Evaluation
                       (Predicate "ICL:preproof")
                       (List
                         (DontExec (Variable "$A"))
                         (Variable "$T")))))
           (results (icr-bc target #:vardecl vardecl)))
      ;; Copy inference control rules to the Inference Control Rules
      ;; atomspace.
      (icl-logger-debug "Results:\n~a" results)
      (cog-cp (cog-outgoing-set results) icr-as))))

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, tr-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target tr-as icr-as i j)
  (icl-logger-info "Run BC (i=~a/~a,j=~a/~a) with target:\n~a"
                   (+ i 1) niter (+ j 1) pss target)
  (reload)
  (let* ((result (pln-bc target #:trace-as tr-as #:control-as icr-as))
         (result-size (length (cog-outgoing-set result)))
         (success (if (= 1 result-size)
                      (tv->bool (cog-tv (gar result)))
                      #f)))
    (icl-logger-info (if success "Success" "Failure"))
    success))

