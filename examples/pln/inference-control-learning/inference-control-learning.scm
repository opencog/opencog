;; Contain the main inference control learning experiment loop

(load "utilities.scm")

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "rb.scm"))

;; Set the random seed of the experiment
(cog-randgen-set-seed! 0)

;; Set loggers levels
(cog-logger-set-level! (cog-ure-logger) "debug")
(cog-logger-set-level! icl-logger "debug")



;; Set parameters
(define pss 10)                         ; Problem set size
(define niter 3)                        ; Number of iterations

(define (run-experiment)
  (icl-logger-info "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (ih-as (cog-new-atomspace))        ; Initialize the Global
                                            ; Inference History
         (ic-rules '()))                    ; Initialize Inference
                                            ; Control Rules
    ;; Function for running all iterations, the inference control
    ;; rules, icr, and the iteration index, i. Return the list of
    ;; solved problems for each iteration (list of list).
    (define (run-iterations-rec icr i)
      (if (< i niter)
          (let* ((sol-icr (run-iteration targets ih-as icr i)))
            (cons (cadr sol-icr) (run-iterations-rec (car sol-icr) (+ i 1))))
          '()))
    ;; Run all iterations
    (run-iterations-rec ic-rules 0)
    (icl-logger-debug "Inference History AtomSpace:")
    (icl-logger-debug-atomspace ih-as)))

;; Run iteration i over the given targets. Return a pair
;;
;; (solved problems, inference control rules)
;;
;; The idea is to pass the inference control rules to the next
;; iteration.
(define (run-iteration targets ih-as ic-rules i)
  (icl-logger-info "Run iteration ~a/~a" (+ i 1) niter)
  (let* (;; Run the BC and build the inference history corpus for that run
         (run-bc-mk-corpus (lambda (j)
                             (let* (;; AtomSpace where to the record
                                    ;; the inference traces for that run
                                    (tr-as (cog-new-atomspace))
                                    ;; Target
                                    (trg (list-ref targets j))
                                    ;; Run the BC and put the trace in tr-as
                                    (bc-result (run-bc trg tr-as ic-rules i j)))
                               ;; Post-process tr-as and copy the
                               ;; relevant knowledge in global-ih-as
                               (postprocess-corpus tr-as ih-as)
                               bc-result)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    (icl-logger-info "Number of problems solved = ~a" sol_count)

    ;; Build inference control rules for the next iteration
    (list sol_count (mk-ic-rules ih-as))))

;; Post-process the trace tr-as by inferring knowledge about preproof,
;; and add all relevant knowledge to the inference history ih-as from
;; it, leaving out cruft like ppc-kb and such.
(define (postprocess-corpus tr-as ih-as)
  (icl-logger-info "Post-process the trace and copy it to the inference history")
  (let ((default-as (cog-set-atomspace! tr-as)))
    ;; Copy the content of tr-as to ih-as (disabled for now)
    ;; (cog-cp-all ih-as)
    ;; Load the knowledge and rule bases
    (load "ppc-kb.scm")
    (load "ppc-rb.scm")
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
      ;; Copy post-processed inference traces to the inference history
      (icl-logger-debug "Results:\n~a" results)
      (cog-cp (cog-outgoing-set results) ih-as))
    (cog-set-atomspace! default-as)))

(define (mk-ic-rules ih-as)
  (icl-logger-info "Build inference control rules from the inference history")
  (let ((default-as (cog-set-atomspace! ih-as)))
    (cog-set-atomspace! default-as))
  ;; TODO infer ic-rules
)

;; Run the backward chainer on target, given the atomspace where to
;; record the inference traces, tr-as, and inference-control rules
;; used for guidance, ic-rules, with for jth target in iteration
;; i. Return #t iff target has been successfully proved.
(define (run-bc target tr-as ic-rules i j)
  (icl-logger-info "Run BC with target = ~a" target)
  (reload)
  (let* ((result (pln-bc target #:trace-as tr-as)) ; TODO use ic-rules
         (result-size (length (cog-outgoing-set result)))
         (default-as (cog-set-atomspace! tr-as)))
    (if (= 1 result-size)
        (tv->bool (cog-tv (gar result)))
        #f)))
