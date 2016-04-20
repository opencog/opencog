; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (srfi srfi-1)) ; For `append-map`

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "action-selector.scm")
(load "demand.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define-public (psi-rule context action demand-goal a-stv demand-node)
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
    to be taken. These are atoms that should be evaluated to return
    TRUE_TV/FASLE_TV.

  action:
  - It should be an atom that can be run by `cog-evaluate!`. That means that
    it will have to return TRUE_TV or FALSE_TV. Any atom that could be executed
    by running `(cog-execute! your-action)`.

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

; --------------------------------------------------------------
(define-public (psi-get-rules demand-node)
"
  Returns a list of all psi-rules that are affect the given demand.

  demand-node:
  - The node that represents the demand.
"
    (cog-chase-link 'MemberLink 'ImplicationLink demand-node)
)

; --------------------------------------------------------------
(define-public (psi-get-all-rules)
"
  Returns a list of all openpsi rules.
"
    (fold append '()
        (par-map (lambda (x) (cog-chase-link 'MemberLink 'ImplicationLink x))
            (cog-outgoing-set (psi-get-all-demands))))
)

; --------------------------------------------------------------
(define-public (psi-rule? atom)
"
  Returns `#t` or `#f` depending on whether the passed argument is a psi-rule
  or not. An ImplicationLink that is a member on of the demand sets is a
  psi-rule.

  atom:
  - An atom passed for checking.
"
    (if (member atom (psi-get-all-rules)) #t #f)
)

; --------------------------------------------------------------
(define-public (psi-get-all-actions)
"
  Returns a list of all openpsi actions.
"
    (cog-outgoing-set (cog-execute! (GetLink
        (MemberLink (VariableNode "x")
        (ConceptNode (string-append (psi-prefix-str) "action")))))))

; --------------------------------------------------------------
(define-public (psi-action? atom)
"
  Check if the given atom is an atom and return `#t` if it is and `#f` either
  wise.

  atom:
  - An atom to be checked whether it is an action or not.
"
    (if (member atom (psi-get-all-actions)) #t #f)
)

; --------------------------------------------------------------
(define (get-c&a impl-link)
"
  Get context and action list from ImplicationLink.
"
    (cog-outgoing-set (list-ref (cog-outgoing-set impl-link) 0))
)

; --------------------------------------------------------------
(define-public (psi-get-context rule)
"
  Get the context of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    (remove psi-action? (get-c&a rule))
)

; --------------------------------------------------------------
(define-public (psi-get-action rule)
"
  Get the action of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    (car (filter psi-action? (get-c&a rule)))
)

; --------------------------------------------------------------
(define-public (psi-get-goal rule)
"
  Get the goal of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    ; NOTE: Why this function? -> For consisentency and to accomodate future
    ; changes
     (cadr (cog-outgoing-set rule))
)

; --------------------------------------------------------------
(define-public (psi-related-goals action)
"
  Return a list of all the goals that are associated by an action. Associated
  goals are those that are the implicand of the psi-rules that have the given
  action in the implicant.

  action:
  - An action that is part of a psi-rule.
"
    (let* ((and-links (cog-filter 'AndLink (cog-incoming-set action)))
           (rules (filter psi-rule? (append-map cog-incoming-set and-links))))
           (delete-duplicates (map psi-get-goal rules))
    )

)

; --------------------------------------------------------------
(define-public (psi-satisfiable? rule)
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

; --------------------------------------------------------------
(define-public (psi-get-satisfiable demand-node)
"
  Returns a list of psi-rules of the given demand that are satisfiable.

  demand-node:
  - The node that represents the demand.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define-public (psi-default-action-selector a-random-state)
"
  Retruns a list of one of the most weighted and satisfiable psi-rules. A single
  psi-rule is returned so as help avoid mulitple actions of the same effect or
  type(aka semantic of the action) from being executed. If a satisfiable rule
  doesn't exist then the empty list is returned.

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

    (let* ((set-link (psi-get-all-demands))
           (demands (cog-outgoing-set set-link))
           (rules (append-map choose-one-rule demands)))
        (if (null? rules)
            '()
            (list (list-ref rules (random (length rules) a-random-state)))
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-select-rules)
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
; --------------------------------------------------------------
(define psi-do-run-loop #t)

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
 behavior-tree-loop-count

 Return the loop-count of the behavior tree.
"
    psi-loop-count
)

; --------------------------------------------------------------
(define-public (psi-run-continue?)  ; public only because its in a GPN
    (set! psi-loop-count (+ psi-loop-count 1))

    ; Pause for 101 millisecs, to kepp the number of loops within a reasonable
    ; range.    ;
    (usleep 101000)
    (if psi-do-run-loop (stv 1 1) (stv 0 1))
)

; ----------------------------------------------------------------------
(define-public (psi-step)
"
  The main function that defines the steps to be taken in every cycle.
"
    (let* ((rules (psi-select-rules)))
        (map (lambda (x)
                (let* ((action (psi-get-action x))
                       (goals (psi-related-goals action)))
                    (cog-execute! action)
                    (map cog-evaluate! goals)))
            rules)
        (stv 1 1)
    )
)

; --------------------------------------------------------------
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

; --------------------------------------------------------------
(define-public (psi-halt)
"
  Tells the psi loop thread, that is started by running `(psi-run)`, to exit.
"
    (set! psi-do-run-loop #f)
)
