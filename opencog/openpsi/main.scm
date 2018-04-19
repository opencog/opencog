;
; main.scm
;
; Defines the method to single-step the rule engine, and the main loop
; to call it.
;
; Copyright (C) 2015-2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud

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
(define* (psi-component name #:optional step)
"
  psi-component NAME STEP
    Create and return a ConceptNode that represents an OpenPsi engine driven
    component called NAME. It associates an action-selector, and a psi-step
    loop. If evaluatable atom, STEP, that encodes what needs to be
    done during each loop, is passed then the loop will evaluate STEP during
    each cycle. If STEP is not passed then the loop will use psi-step.
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
          (if step
            step
            (Evaluation
              (GroundedPredicate "scm: psi-step")
              (List component)))
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
