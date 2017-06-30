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
(define pss 10)                         ; Problem set size
(define niter 10)                       ; Number of iteration

(define (run-experiment)
  (cog-logger-info icl-logger "Start experiment")
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (runiter (lambda (i) (run-iteration targets i))))
    ;; Run all iterations
    (map runiter (iota niter))))

;; Run an iteration i over the given targets
(define (run-iteration targets i)
  (cog-logger-info icl-logger "Run iteration ~a/~a" (+ i 1) niter)
  (let* ((run-bc-fun (lambda (j) (run-bc (list-ref targets j) i j)))
         (results (map run-bc-fun (iota pss)))
         (Si (count values results)))
    (cog-logger-info icl-logger "Number of problem solved = ~a" Si))
  ;; TODO: build collection of rules for the next iteration
)

;; Run the backward chainer on target, with index j in iteration
;; i. Return #t iff successful.
(define (run-bc target i j)
  (cog-logger-info icl-logger "Run BC with target = ~a" target)
  (reload)
  (let* ((result (pln-bc target)))
    ;; (cog-logger-info icl-logger "result = ~a" result)
    ;; (cog-logger-info icl-logger "Target after run = ~a" target)
    (tv->bool (cog-tv (gar result)))))     ; For some strange reason the
                                           ; target tv doesn't get
                                           ; updated so we take first
                                           ; result instead.
