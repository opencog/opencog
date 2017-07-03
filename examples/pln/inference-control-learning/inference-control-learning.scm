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
  (cog-logger-info icl-logger "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (tr-as (cog-new-atomspace))        ; Trace AtomSpace
         (runiter (lambda (i) (run-iteration targets tr-as i))))
    ;; Run all iterations
    (map runiter (iota niter))))

;; Run an iteration i over the given targets
(define (run-iteration targets tr-as i)
  (cog-logger-info icl-logger "Run iteration ~a/~a" (+ i 1) niter)
  (let* ((ic-kb '())
         (run-bc-fun (lambda (j) (run-bc (list-ref targets j) ic-kb tr-as i j)))
         (results (map run-bc-fun (iota pss)))
         (Si (count values results)))
    (cog-logger-info icl-logger "Number of problem solved = ~a" Si))
  ;; TODO: parse the log file to get all the data. Basically fill the
  ;; preproof(A, T) relationships

  ;; TODO: build collection of rules for the next iteration
)

;; Run the backward chainer on target, given the
;; inference-control-knowledge-base (ic-kb), recording the trace on
;; tr-as, with index j in iteration i. Return #t iff successful.
(define (run-bc target ic-kb tr-as i j)
  (cog-logger-info icl-logger "Run BC with target = ~a" target)
  (reload)
  (let* ((result (pln-bc target #:trace-as tr-as))
         (result-size (length (cog-outgoing-set result)))
         (former-as (cog-atomspace)))
    (cog-set-atomspace! tr-as)
    (cog-prt-atomspace)
    (cog-set-atomspace! former-as)
    (if (= 1 result-size)
        (tv->bool (cog-tv (gar result)))
        #f)))
