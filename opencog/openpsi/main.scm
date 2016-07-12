; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (srfi srfi-1)) ; For `append-map`

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

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
  Return #t if the openpsi loop is running, else return #f.
"
    psi-do-run-loop
)

; --------------------------------------------------------------
(define psi-loop-count 0)

(define-public (psi-get-loop-count)
"
  Returns how many times psi-step have been executed.
"
    psi-loop-count
)

; --------------------------------------------------------------
(define-public (psi-run-continue?)  ; public only because its in a GPN
"
  Function used for checking whether the thread that is executing the
  psi-loop should continue doing so.
"
    (set! psi-loop-count (+ psi-loop-count 1))

    ; Pause for 101 millisecs, to keep the number of loops
    ; within a reasonable range.
    (usleep 101000)
    (if psi-do-run-loop (stv 1 1) (stv 0 1))
)

; ----------------------------------------------------------------------
(define-public (psi-step)
"
  The main function that defines the steps to be taken in every cycle.
"
; TODO: Add reinforcement signal in psi-step
; 1. Record which rule-was selected per demand on the previous run.
; 2. Update the rule strength depndeing on the reinforcement signal.
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

            (if (null? context-atoms)
                (cog-execute! action)
                (cog-execute! (PutLink action context-atoms))
            )
            (map cog-evaluate! goals)
        ))

    ; Run the controler that updates the weight.
    (psi-controller-update-weights)

    ; Do action-selection & orchesteration.
    (map
        (lambda (d)
            (let ((updater (psi-get-updater d)))
                ; Run the updater for the demand.
                (if (not (null? updater))
                    (cog-evaluate! updater)
                )
                ; The assumption is that the rules can be run concurrently.
                ; FIXME: Once action-orchestrator is available then a modified
                ; `psi-select-rules` should be used insted of
                ; `psi-select-rules-per-demand`
                (map act-and-evaluate (psi-select-rules-per-demand d))
            ))

        (psi-get-all-valid-demands)
    )

    (stv 1 1) ; For continuing psi-run loop.
)

; --------------------------------------------------------------
(define-public (psi-run)
"
  Run `psi-step` in a new thread. Call (psi-halt) to exit the loop.
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

    (set! psi-do-run-loop #t)
    (call-with-new-thread
        (lambda () (cog-evaluate! loop-node)))
)

; -------------------------------------------------------------
(define-public (psi-halt)
"
  Tells the psi loop thread, that is started by running `(psi-run)`, to exit.
"
    (set! psi-do-run-loop #f)
)
