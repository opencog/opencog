;
; main.scm
;
; Defines the method to single-step the rule engine, and the main loop
; to call it.
;
; Copyright (C) 2015-2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud

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
; Configure openpsi logger
(define opl (cog-new-logger))
(cog-logger-set-component! opl "OpenPsi")
(cog-logger-set-level! opl "debug")
(cog-logger-set-stdout! opl #f)

(define (psi-get-logger)
"
  psi-get-logger

  Returns the looger for openpsi.
"
  opl
)

; --------------------------------------------------------------
; Define the component category. Components are like mind-agents
; but there activities are defined using openpsi-rules and action-selectors
; associated with it.
(define psi-component-node (ConceptNode "component"))
(psi-add-category psi-component-node)

; --------------------------------------------------------------
(define (psi-component name)
"
  psi-component NAME
    Create and return a ConceptNode that represents an OpenPsi engine driven
    component called NAME. It associates an action-selector, and a psi-step
    loop.
"
  ; NOTE: All the values associated with the component can easily be
  ; moved into the atomspace.
  (let ((component (ConceptNode name))
    (loop-node (DefinedPredicate (string-append name "-loop")))
    )
    (InheritanceLink component psi-component-node)

    ; Assign a default action-selector
    (psi-set-action-selector! component
      (ExecutionOutput
        (GroundedSchema "scm: psi-get-satisfiable-rules")
        (List component))
    )

    ; Add a value for controlling whether to keep on running the loop or not.
    (cog-set-value! component (Predicate "run-loop") (StringValue "#f"))

    ; Add a value for counting the number of times the psi-step has
    ; been executed
    (cog-set-value! component (Predicate "loop-count") (FloatValue 0))

    ; Define the loop that would be run. This is stored as value so as
    ; to save on searching for it in the atomspace and make it accessible
    ; through any programming language.
    (Define
      loop-node
      (Satisfaction
        (SequentialAnd
          (Evaluation
            (GroundedPredicate "scm: psi-step")
            (List component))
          (Evaluation
            (GroundedPredicate "scm: psi-run-continue?")
            (List component))
          ; tail-recursive call
          loop-node)))

    (cog-set-value! component (Predicate "loop") loop-node)
    component
  )
)

; --------------------------------------------------------------
;
; XXX FIXME -- right now, this assumes that a single thread, running
; at no more than 100 steps per second, is sufficient to run all of the
; psi rules.  For now, this is OK, but at some point, this will become
; a bottleneck, as we will need to evaluate more rules more often.
;
(define (psi-run component)
"
  psi-run COMPONENT

  Create a new thread, and repeatedly invoke `psi-step` in it.
  This thread can be halted by calling `(psi-halt COMPONENT)`, which will exit
  the loop, and kill the thread.
"
  (if (not (psi-running? component))
    (begin
      (cog-set-value! component (Predicate "run-loop") (StringValue "#t"))
      (call-with-new-thread
        (lambda () (cog-evaluate! (cog-value component (Predicate "loop"))))))
  )
)

; --------------------------------------------------------------
(define (psi-running? component)
"
  psi-running? COMPONENT

  Return #t if the openpsi loop of COMPONENT is running, else return #f.
"
  (equal? "#t"
    (cog-value-ref (cog-value component (Predicate "run-loop")) 0))
)

; --------------------------------------------------------------
(define (psi-run-continue? component)  ; public because its in a GPN
"
  psi-run-continue? COMPONENT

  Return TRUE_TV if the psi loop of COMPONENT should continue running,
  else returns FALSE_TV.
"
    ; Pause for 10 millisecs, so that the psi engine doesn't hog
    ; all CPU. FIXME -- this is obviously a hack, awaiting some sort
    ; of better way of scehduling psi rules.
    (usleep 10000)
    (if (psi-running? component) (stv 1 1) (stv 0 1))
)

; -------------------------------------------------------------
(define (psi-halt component)
"
  psi-halt COMPONENT

  Halts COMPONENT's previously-started psi loop thread. The thread is
  started by calling `(psi-run COMPONENT)`.
"
  (cog-set-value! component (Predicate "run-loop") (StringValue "#f"))
)

; --------------------------------------------------------------
(define (psi-loop-count component)
"
  psi-loop-count COMPONENT

  Returns the number of times that psi-step has been executed.
"
  (cog-value-ref (cog-value component (Predicate "loop-count")) 0)
)

; ----------------------------------------------------------------------
(define (psi-step component)
"
  psi-step COMPONENT

  Take one step of the OpenPsi rule engine COMPONENT.
  Returns TRUE_TV, always.
"
  (define (psi-act rule)
    ; The rules passed in are result from the action-selector associated
    ; with the component. This is here only for logging.
    (cog-logger-debug opl "In component ~a starting evaluation of ~a"
      component rule)
    (psi-imply rule)
    (cog-logger-debug opl "In component ~a finished evaluation of ~a"
      component rule))

  (let ((lc (psi-loop-count component)))
    (cog-set-value! component (Predicate "loop-count") (FloatValue (+ lc 1)))

    (cog-logger-debug opl
      "In component ~a taking one psi-step, loop-count = ~a" component lc)

    ; Do action-selection and action-execution.
    (par-map psi-act (psi-select-rules component))

    (cog-logger-debug opl
      "In component ~a ending psi-step, loop-count = ~a" component lc)
    (stv 1 1) ; For continuing psi-run loop.
  )
)

; ----------------------------------------------------------------------
(define (psi-step-per-demand)
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
            (cog-extract pattern)
            ; If it is only links then nothing to pass to an action.
            (if (null? (cog-get-all-nodes results))
                '()
                results
            )))!#
            '())


    (define (act-and-evaluate rule)
        ;NOTE: This is the job of the action-orchestrator.
        (let* ((action (psi-get-action rule))
              ; (goals (psi-related-goals action))
               (context-atoms (get-context-grounding-atoms rule)))

            (cog-logger-debug opl "Starting evaluation of psi-rule ~a" rule)

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
            ; NOTE: The evalution of goals is disabled as it isn't being used
            ; and is a candidate to be refactored out.
            ;(map cog-evaluate! goals)
            (cog-logger-debug opl "Finished evaluating of psi-rule ~a" rule)
        ))

    (set! psi-loop-count (+ psi-loop-count 1))

    (cog-logger-debug opl "Taking one psi-step, loop-count = ~a" psi-loop-count)

    ; Run the controller that updates the weight.
    ; TODO: Should this be a before selection hook? The real reason is that
    ; the other components might want to specify other actions to be
    ; undertaken.
    (psi-controller-update-weights)

    ; Do action-selection.
    (map
        (lambda (d)
         ;TODO: Replace the updater with a hook? Maybe using Join/Parallel
         ; Links?
            (let ((updater (psi-get-updater d)))
                ; Run the updater for the demand.
                (if (not (null? updater))
                    (cog-evaluate! updater)
                )
                ; The assumption is that the rules can be run concurrently.
                (par-map act-and-evaluate (psi-select-rules d))
            ))

        (psi-get-all-enabled-demands)
    )

    ; Do garbage collection. This is a replacement to (run-behavior-tree-gc)
    ; TODO: Each component must clean after itself or have a separate
    ; component that does that. So, remove this. Should this be an after
    ; selection hook?
    (when (equal? 0 (modulo psi-loop-count 1000))
        (cog-map-type (lambda (a) (cog-extract a) #f) 'SetLink)
        (cog-map-type (lambda (a) (cog-extract a) #f) 'ListLink)
        (cog-map-type (lambda (a) (cog-extract a) #f) 'NumberNode)
        (cog-map-type (lambda (a) (cog-extract a) #f) 'ConceptNode)
        (cog-logger-debug opl
            "Finished garbage collection, loop-count = ~a" psi-loop-count)
    )

    (cog-logger-debug opl "Ending psi-step, loop-count = ~a" psi-loop-count)
    (stv 1 1) ; For continuing psi-run loop.
)

; --------------------------------------------------------------
;
; XXX FIXME -- right now, this assumes that a single thread, running
; at no more than 100 steps per second, is sufficient to run all of the
; psi rules.  For now, this is OK, but at some point, this will become
; a bottleneck, as we will need to evaluate more rules more often.
;
(define (psi-run-per-demand)
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
