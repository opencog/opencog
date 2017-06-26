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

;; Define
(define (run-iteration targets i)
  (display "i = ") (display i) (display "\n"))
