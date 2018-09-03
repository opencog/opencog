;; Contain the main inference control learning experiment loop

;; Load utils
(load "icl-parameters.scm")
(load "icl-utilities.scm")
(load "mk-history.scm")
(load "mk-control-rules.scm")
(load "mine-control-rules.scm")

;; Set the random seed of the experiment
(cog-randgen-set-seed! 1)

;; Set loggers levels
(cog-logger-set-level! "info")
(icl-logger-set-level! "debug")
(ure-logger-set-level! "info")

;; Set loggers stdout
;; (cog-logger-set-stdout! #t)
(icl-logger-set-stdout! #t)
;; (ure-logger-set-stdout! #t)

;; Set loggers sync (for debugging)
(cog-logger-set-sync! #t)
(icl-logger-set-sync! #t)
(ure-logger-set-sync! #t)

;; Clear and reload the kb and rb
(define (reload)
  (clear)
  (load "kb.scm")
  (load "pln-rb.scm"))

;; AtomSpace containing the targets in there to no forget them
(define targets-as (cog-new-atomspace))

(define (run-experiment)
  (icl-logger-info "Start experiment")

  (let* ((default-as (cog-set-atomspace! targets-as)) ; Switch to targets-as
         (targets (gen-random-targets pss))) ; Generate targets

    ;; Switch back to the default atomspace
    (cog-set-atomspace! default-as)

    ;; Run all iterations
    (map (lambda (i) (run-iteration targets i)) (iota niter))))

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

    (icl-logger-info "Number of solved problems = ~a/~a" sol_count pss)

    ;; Copy all atomspaces for histories to history-as
    (icl-logger-info "Move all problem histories to history-as")
    (union-as history-as histories)

    ;; Remove dangling atoms from history-as. These are produced due
    ;; to alpha-conversion. It creates inconsistencies, see
    ;; https://github.com/opencog/atomspace/issues/1417. It's not too
    ;; harmful for now but it will have to be remedied at some point.
    (icl-logger-info "Remove dangling atoms from history-as")
    (remove-dangling-atoms history-as)

    ;; Build inference control rules for the next iteration
    (icl-logger-info "Build inference control rules from history-as")
    (mk-control-rules)

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
