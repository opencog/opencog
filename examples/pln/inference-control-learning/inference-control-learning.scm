;; Contain the main inference control learning experiment loop

(load "utilities.scm")

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "rb.scm"))

;; Set the random seed of the experiment
(cog-randgen-set-seed! 0)

;; Set ure logger to debug
(cog-logger-set-level! (cog-ure-logger) "debug")

;; Set parameters
(define pss 2)                         ; Problem set size
(define niter 2)                       ; Number of iterations

(define (run-experiment)
  (icl-logger-info "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (tr-as (init-tr-as))               ; Initialize Trace AtomSpace
         (ic-rules '()))                    ; Initialize Inference
                                            ; Control Rules
    ;; Function for running all iterations, the inference control
    ;; rules, icr, and the iteration index, idx. Return the list of
    ;; solved problems for each iteration (list of list).
    (define (run-iterations-rec icr i)
      (if (< idx niter)
          (let* ((sol-icr (run-iteration targets tr-as icr i)))
            (cons (cadr sol-icr) (run-iterations-rec (car col-icr) (+ i 1))))
          '()))
    ;; Run all iterations
    (run-iterations-rec ic-rules 0)))

;; Initialize the trace atomspace with the knowledge and rule bases
;; for inferring the corpus and the inference control rules.
(define (init-tr-as)
  (let* ((tr-as (cog-new-atomspace))    ; Create trace AtomSpace
         (old-as (cog-set-atomspace! tr-as))) ; Set it as current AtomSpace
    ;; Load meta-kb and meta-rb containing the axioms the rule bases
    ;; to infer inference control rules
    (load "meta-kb.scm")
    (load "meta-rb.scm")
    ;; Switch back to the old atomspace and return tr-as
    (cog-set-atomspace! old-as)))

;; run iteration i over the given targets. Return a pair
;;
;; (solved problems, inference control rules)
;;
;; The idea is to pass the inference control rules to the next
;; iteration.
(define (run-iteration targets tr-as ic-rules i)
  (icl-logger-info "Run iteration ~a/~a" (+ i 1) niter)
  (let* (;; Run the BC and build the trace corpus
         (run-bc-mk-corpus (lambda (j)
                             (run-bc (list-ref targets j) tr-as ic-rules i j)
                             (postprocess-corpus tr-as)))
         (results (map run-bc-mk-corpus (iota pss)))
         (sol_count (count values results)))
    (icl-logger-info "Number of problem solved = ~a" sol_count))

  ;; Build inference control rules for the next iteration
  (list sol_count (mk-ic-rules tr-as)))

(define (postprocess-corpus tr-as)
  ;; TODO infer preproof
)

(define (infer-ic-rules tr-as)
  ;; TODO infer ic-rules
)

;; Run the backward chainer on target, given the current trace
;; atomspace, tr-as, and inference-control rules, ic-rules, adding the
;; new traces to tr-as, with index j in iteration i. Return #t iff
;; target has been successfully proved.
(define (run-bc target tr-as ic-rules i j)
  (icl-logger-info "Run BC with target = ~a" target)
  (reload)
  (let* ((result (pln-bc target #:trace-as tr-as)) ; TODO use ic-rules
         (result-size (length (cog-outgoing-set result)))
         (former-as (cog-atomspace)))
    (if (= 1 result-size)
        (tv->bool (cog-tv (gar result)))
        #f)))
