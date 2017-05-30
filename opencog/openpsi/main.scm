;
; main.scm
;
; Defines the method to single-step the rule engine, and the main loop
; to call it.
;
; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (srfi srfi-1)) ; For `append-map`
(use-modules (ice-9 threads)) ; For par-map

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))
(use-modules (opencog logger))

(load "action-selector.scm")
(load "demand.scm")
(load "control.scm")
(load "rule.scm")
(load "utilities.scm")

; --------------------------------------------------------------
; Variable for controlling whether to keep on running the loop or not.
(define psi-do-run-loop #f)

; --------------------------------------------------------------
(define-public (psi-running?)
"
  psi-running?

  Return #t if the openpsi loop is running, else return #f.
"
    psi-do-run-loop
)

; --------------------------------------------------------------
(define psi-loop-count 0)

(define-public (psi-get-loop-count)
"
  psi-get-loop-count

  Returns the number of times that psi-step has been executed.
"
    psi-loop-count
)

; --------------------------------------------------------------
(define-public (psi-run-continue?)  ; public only because its in a GPN
"
  psi-run-continue?

  Return TRUE_TV if the the main psi loop should continue running,
  else returns FALSE_TV.
"
    ; Pause for 10 millisecs, so that the psi engine doesn't hog
    ; all CPU. FIXME -- this is obviously a hack, awaiting some sort
    ; of better way of scehduling psi rules.
    (usleep 10000)
    (if psi-do-run-loop (stv 1 1) (stv 0 1))
)

; ----------------------------------------------------------------------
(define-public (psi-step)
"
  psi-step - Take one step of the OpenPsi rule engine.

  Returns TRUE_TV, always.
"
; TODO: Add reinforcement signal in psi-step
; 1. Record which rule was selected, per demand, on the previous run.
; 2. Update the rule strength, depending on the reinforcement signal.
; 3. Define the representation for the reinforcement signal.
    (define (get-context-grounding-atoms rule)
        #!
        (let* ((pattern (GetLink (AndLink (psi-get-context rule))))
                ;FIXME: Cache `results` during `psi-select-rules` stage
               (results (cog-execute! pattern)))
            (cog-delete pattern)
            ; If it is only links then nothing to pass to an action.
            (if (null? (cog-get-all-nodes results))
                '()
                results
            )))!#
            '())


    (define (act-and-evaluate rule)
        ;NOTE: This is the job of the action-orchestrator.
        (let* ((action (psi-get-action rule))
               (goals (psi-related-goals action))
               (context-atoms (get-context-grounding-atoms rule)))

            (cog-logger-debug "[OpenPsi] Starting evaluation of psi-rule ~a"
                rule)

            ; The #t condition is for evaluatable-contexts. These are
            ; contexts that only have evaluatable-terms that return TRUE_TV
            ; or FALSE_TV.
            ; The #f condition is for groundable-contexts. These are contexts,
            ; that are similar to the implicant of a BindLink. The contexts are
            ; grounded and the grounding atoms are put into the action (that is
            ; equivalent to the implicand of the BindLink).
            (if (null? context-atoms)
                (cog-evaluate! action)
                ; FIXME Since the PutLink is wrapped in a TrueLink any
                ; information due to the evaluation of the action is lost.
                (cog-evaluate! (True (PutLink action context-atoms)))
            )
            ; An evaluation of an action that is common in mulitple rules
            ; results in the achievement of the goals, even if the context of
            ; the other rules aren't not satisfied.
            (map cog-evaluate! goals)
            (cog-logger-debug "[OpenPsi] Finished evaluating of psi-rule ~a"
                rule)
        ))

    (set! psi-loop-count (+ psi-loop-count 1))

    (cog-logger-debug "[OpenPsi] Taking one psi-step, loop-count = ~a"
        psi-loop-count)

    ; Run the controller that updates the weight.
    (psi-controller-update-weights)

    ; Do action-selection.
    (map
        (lambda (d)
            (let ((updater (psi-get-updater d)))
                ; Run the updater for the demand.
                (if (not (null? updater))
                    (cog-evaluate! (car updater))
                )
                ; The assumption is that the rules can be run concurrently.
                (par-map act-and-evaluate (psi-select-rules-per-demand d))
            ))

        (psi-get-all-enabled-demands)
    )

    (cog-logger-debug "[OpenPsi] Ending psi-step, loop-count = ~a"
        (psi-get-loop-count))

    (stv 1 1) ; For continuing psi-run loop.
)

; --------------------------------------------------------------
;
; XXX FIXME -- right now, this assumes that a single thread, running
; at no more than 100 steps per second, is sufficient to run all of the
; psi rules.  For now, this is OK, but at some point, this will become
; a bottleneck, as we will need to evaluate more rules more often.
;
(define-public (psi-run)
"
  psi-run

  Create a new thread, and repeatedly invoke `psi-step` in it.
  This thread can be halted by calling `(psi-halt)`, which will exit
  the loop (and kill the thread).
"
    (define loop-name (string-append psi-prefix-str "loop"))
    (define loop-node (DefinedPredicateNode loop-name))
    (define (define-psi-loop)
        (DefineLink
            loop-node
            (SatisfactionLink
                (SequentialAnd
                    (Evaluation
                        (GroundedPredicate "scm: psi-step")
                        (ListLink))
                    (Evaluation
                        (GroundedPredicate "scm: psi-run-continue?")
                        (ListLink))
                    ; tail-recursive call
                    loop-node))))

    (if (or (null? (cog-node 'DefinedPredicateNode loop-name))
            (null? (cog-chase-link 'DefineLink 'SatisfactionLink loop-node)))
        (define-psi-loop))

    (if (not (psi-running?))
        (begin
            (set! psi-do-run-loop #t)
            (call-with-new-thread (lambda () (cog-evaluate! loop-node)))))
)

; -------------------------------------------------------------

(define-public (psi-halt)
"
  psi-halt

  Halts a previously-started psi loop thread. The thread is started
  by calling `(psi-run)`.
"
    (set! psi-do-run-loop #f)
)
