; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (srfi srfi-1)) ; For set-difference

(use-modules (opencog) (opencog exec) (opencog rule-engine))

(load-from-path "openpsi/action-selector.scm")
(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define-public (psi-step)
"
  The main function that defines the steps to be taken in every cycle.
"
    (let* ((rules (psi-select-rules)))
        (map (lambda (x)
                (let* ((action (psi-get-action x))
                       (goals (append-map (lambda (r)
                            (cog-chase-link 'ImplicationLink 'EvaluationLink r))
                            (cog-incoming-set action))))
                    (cog-execute! action)
                    (map cog-evaluate! goals)))
            rules)
        (stv 1 1)
    )
)

; --------------------------------------------------------------
(define (psi-rule context action demand-goal a-stv demand-node)
"
  It associates an action and context in which the action has to be taken
  to a goal that is satisfied when the action is executed. It is structured as
  as the following `ImplicationLink`,

    (ImplicationLink
        (AndLink
            (context)
            (action))
        (demand-goal))

  context:
  - A list containing the terms/clauses that should be met for this action
    to be taken.

  action:
  - It should be an atom that can be run by `cog-evaluate!`. That means that
    it will have to return TRUE_TV or FALSE_TV.

  demand-goal:
  - It should be an atom that can be run by `cog-evaluate!`. That means that
  it will have to return TRUE_TV or FALSE_TV. This is basically a formula, on
  how this rule affects the demands.

  a-stv:
  - This is the stv of the ImplicationLink.

  demand-node:
  - The node that represents a demand that this rule affects.
"
    (define func-name "psi-rule") ; For use in error reporting
    (define (implication)
        (ImplicationLink a-stv (AndLink context action) demand-goal))

    ; Check arguments
    (if (not (list? context))
        (error (string-append "In procedure " func-name ", expected first "
            "argument to be a list, got:") context))
    (if (not (cog-atom? action))
        (error (string-append "In procedure " func-name ", expected second "
            "argument to be an atom, got:") action))
    (if (not (cog-atom? demand-goal))
        (error (string-append "In procedure " func-name ", expected third "
            "argument to be an atom, got:") demand-goal))
    (if (not (cog-tv? a-stv))
        (error (string-append "In procedure " func-name ", expected fourth "
            "argument to be a stv, got:") a-stv))
    (if (not (equal? (stv 1 1) (psi-demand? demand-node)))
        (error (string-append "In procedure " func-name ", expected fifth "
            "argument to be a node representing a demand, got:") demand-node))

    ; These memberships are needed for making filtering and searching simpler..
    ; If GlobNode had worked with GetLink at the time of coding this ,
    ; that might have been; better,(or not as it might need as much chasing)
    (MemberLink
        action
        (ConceptNode (string-append (psi-prefix-str) "action")))

    (MemberLink (implication) demand-node)

    (implication)
)

(define (psi-get-all-actions) ; get openpsi actions
    (cog-outgoing-set (cog-execute! (GetLink
        (MemberLink (VariableNode "x")
        (ConceptNode (string-append (psi-prefix-str) "action")))))))

(define (psi-action? x)
    (if (member x (psi-get-all-actions)) #t #f))

(define (psi-get-rules demand-node) ; get all openpsi rules
    (cog-chase-link 'MemberLink 'ImplicationLink demand-node))

(define (psi-get-context rule) ; get the context of an openpsi-rule
    (define (get-c&a x) ; get context and action list from ImplicationLink
        (cog-outgoing-set (list-ref (cog-outgoing-set x) 0)))

    (remove psi-action? (get-c&a rule))
)

(define (psi-get-action rule) ; get the context of an openpsi-rule
    (define (get-c&a x) ; get context and action list from ImplicationLink
        (cog-outgoing-set (list-ref (cog-outgoing-set x) 0)))

    (car (filter psi-action? (get-c&a rule)))
)

(define (psi-get-goal rule)
"
  Get the goal of an openpsi-rule.
"
    ; NOTE: Why this function? -> For consisentency and to accomodate future
    ; changes
     (cadr (cog-outgoing-set rule))
)

(define (psi-satisfiable? rule)
"
  Check if the rule is satisfiable and return TRUE_TV or FALSE_TV.

  rule:
  - A psi-rule to be checked if its context is satisfiable.
"
    ; NOTE: stv are choosen as the return values so as to make the function
    ; usable in evaluatable-terms.
    (let* ((pattern (SatisfactionLink (AndLink (psi-get-context rule))))
           (result (cog-evaluate! pattern)))
          (cog-delete pattern)
          result
    )
)

(define (psi-get-satisfiable demand-node)
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-rules demand-node))
)

(define (psi-default-action-selector a-random-state)
"
  Retruns a list of all most weighted and satisfiable psi-rules.

  a-random-state:
  - A random-state object used as a seed on how psi-rules of a demand, that are
  satisfiable, are to be choosen.
"
    (define (choose-one-rule demand-node)
        ; Returns an empty list or a list containing a randomly choosen
        ; satisfiable psi-rule.
        (let ((rules (most-weighted-atoms (psi-get-satisfiable demand-node))))
            (if (null? rules)
                '()
                (list (list-ref rules (random (length rules) a-random-state)))))
    )

    (let* ((set-link (psi-get-demands-all))
           (demands (cog-outgoing-set set-link)))
       (append-map choose-one-rule demands)
    )
)

(define (psi-select-rules)
"
  Returns a list of psi-rules that are satisfiable by using the action-selector
  you defined or the default-action-selector predefined if you haven't defined
  a different action-selector.
"
    (let ((dsn (psi-get-action-selector)))
        (if (null? dsn)
            (psi-default-action-selector (random-state-from-platform))
            (let ((result (cog-execute! (car dsn))))
                (if (equal? (cog-type result) 'SetLink)
                    (cog-outgoing-set result)
                    (list result)
                )
            )
        )
    )
)

; --------------------------------------------------------------
; Main loop control
(define psi-do-run-loop #t)

(define-public (psi-running?)
"
  Return #t if the openpsi loop is running, else return #f.
"
    psi-do-run-loop
)

(define psi-loop-count 0)

(define-public (psi-get-loop-count)
"
 behavior-tree-loop-count

 Return the loop-count of the behavior tree.
"
    psi-loop-count
)

(define-public (psi-run-continue?)  ; public only because its in a GPN
    (set! psi-loop-count (+ psi-loop-count 1))

    ; Pause for 101 millisecs, to kepp the number of loops within a reasonable
    ; range.    ;
    (usleep 101000)
    (if psi-do-run-loop (stv 1 1) (stv 0 1))
)

; ----------------------------------------------------------------------
(define-public (psi-run)
"
  Run `psi-step` in a new thread. Call (psi-halt) to exit the loop.
"
    (define loop-name (string-append (psi-prefix-str) "loop"))
    (define (loop-node) (DefinedPredicateNode loop-name))
    (define (define-psi-loop)
        (DefineLink
            (loop-node)
            (SatisfactionLink
                (SequentialAnd
                    (Evaluation
                        (GroundedPredicate "scm: psi-step")
                        (ListLink))
                    (Evaluation
                        (GroundedPredicate "scm: psi-run-continue?")
                        (ListLink))
                    (loop-node)))))

    (if (null? (cog-node 'DefinedPredicateNode loop-name))
        (define-psi-loop)
        #f ; Nothing to do already defined
    )
    (set! psi-do-run-loop #t)
    (call-with-new-thread
        (lambda () (cog-evaluate! (loop-node))))
)

(define-public (psi-halt)
"
  Tell the psi main loop thread to exit.
"
    (set! psi-do-run-loop #f)
)
