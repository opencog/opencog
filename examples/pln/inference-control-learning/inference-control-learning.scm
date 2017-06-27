;; Contain the main inference contrl learning experiment loop

(load "utilities.scm")

;; Set the random seed of the experiment
(cog-randgen-set-seed! 0)

;; Set parameters
(define pss 10)                         ; Problem set size
(define niter 10)                       ; Number of iteration

(define (run-experiment)
  (let* ((targets (gen-random-targets pss)) ; Generate targets
         (runiter (lambda (i) (run-iteration targets i))))
    ;; Run all iterations
    (map runiter (iota niter))))

;; Run an iteration i over the given targets
(define (run-iteration targets i)
  (display (format #f "run-iteration i = ~a\n" i))
  (let* ((run-bc-fun (lambda (j) (run-bc (list-ref targets j) i j)))
         (results (map run-bc-fun (iota pss)))
         (Si (count values results)))
    (display (format #f "Number of problem solved = ~a\n" Si))
    (display "TODO: build collection of rules for the next iteration\n")))

;; Run the backward chainer on target, with index j in iteration
;; i. Return #t iff successful.
(define (run-bc target i j)
  (display (format #f "run-bc target = ~a, i = ~a, j = ~a\n" target i j))
  #f)
