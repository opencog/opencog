; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (srfi srfi-1)) ; For `append-map`

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "action-selector.scm")
(load "demand.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define-public (psi-rule-nocheck context action goal a-stv demand)
"
  psi-rule-nocheck -- same as psi-rule, but no checking
"
    ; These memberships are needed for making filtering and searching simpler..
    ; If GlobNode had worked with GetLink at the time of coding this,
    ; that might have been; better, (or not as it might need as much chasing)
    (MemberLink
        action
        (ConceptNode (string-append (psi-prefix-str) "action")))

    (MemberLink (implication) demand)

    (implication)
)

; --------------------------------------------------------------

(define-public (psi-rule context action goal a-stv demand)
"
  It associates an action and context in which the action has to be taken
  to a goal that is satisfied when the action is executed. It is structured as
  as the following `ImplicationLink`,

    (ImplicationLink a-stv
        (AndLink
            (context)
            (action))
        (goal))

  context:
  - A list containing the terms/clauses that should be met for this action
    to be taken. These are atoms that should be evaluated to return
    TRUE_TV/FALSE_TV.

  action:
  - It should be an atom that can be run by `cog-evaluate!`. That means that
    it will have to return TRUE_TV or FALSE_TV. Any atom that could be executed
    by running `(cog-execute! your-action)`.

  goal:
  - It should be an atom that can be run by `cog-evaluate!`. That means that
    it will have to return TRUE_TV or FALSE_TV. This is basically a formula, on
    how this rule affects the demands.

  a-stv:
  - This is the stv of the ImplicationLink.

  demand:
  - The node that represents a demand that this rule affects.
"
    (define func-name "psi-rule") ; For use in error reporting
    (define (implication)
        (ImplicationLink a-stv (AndLink context action) goal))

    ; Check arguments
    (if (not (list? context))
        (error (string-append "In procedure " func-name ", expected first "
            "argument to be a list, got:") context))
    (if (not (cog-atom? action))
        (error (string-append "In procedure " func-name ", expected second "
            "argument to be an atom, got:") action))
    (if (not (cog-atom? goal))
        (error (string-append "In procedure " func-name ", expected third "
            "argument to be an atom, got:") goal))
    (if (not (cog-tv? a-stv))
        (error (string-append "In procedure " func-name ", expected fourth "
            "argument to be a stv, got:") a-stv))
    (if (not (equal? (stv 1 1) (psi-demand? demand)))
        (error (string-append "In procedure " func-name ", expected fifth "
            "argument to be a node representing a demand, got:") demand))

    (psi-rule-nocheck context action goal a-stv demand)
)

; --------------------------------------------------------------
(define-public (psi-get-rules demand-node)
"
  Returns a list of all psi-rules that affect the given demand.

  demand-node:
  - The node that represents the demand.
"
    (cog-chase-link 'MemberLink 'ImplicationLink demand-node)
)

; --------------------------------------------------------------
(define-public (psi-get-all-rules)
"
  Returns a list of all known openpsi rules.
"
    (fold append '()
        (par-map (lambda (x) (cog-chase-link 'MemberLink 'ImplicationLink x))
            (cog-outgoing-set (psi-get-all-demands))))
)

; --------------------------------------------------------------
(define-public (psi-rule? atom)
"
  Returns `#t` or `#f` depending on whether the passed argument is a psi-rule
  or not. An ImplicationLink that is a member of the demand sets is a
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
  Check if the given atom is an action and return `#t`, if it is, and `#f`
  otherwise. An atom is an action if it a member of the set represented by
  (ConceptNode \"OpenPsi: action\").

  atom:
  - An atom to be checked whether it is an action or not.
"
    (let ((action-node
            (cog-node 'ConceptNode (string-append (psi-prefix-str) "action")))
          (candidates (cog-chase-link 'MemberLink 'ConceptNode atom)))

        (if (null? action-node)
            ; `#f` is returned becasue an action hasn't been created yet, thus
            ; the given atom can't be an action.
            #f
            ; A filter is used to account for empty list as well as
            ; cog-chase-link returning multiple results, just in case.
            (not (null?
                (filter (lambda (x) (equal? x action-node)) candidates)))
        )
    )
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
(define-public (psi-get-satisfiable-rules demand-node)
"
  Returns a list of psi-rules of the given demand that are satisfiable.

  demand-node:
  - The node that represents the demand.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define-public (psi-get-all-satisfiable-rules)
"
  Returns a list of all the psi-rules that are satisfiable.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-all-rules))
)

; --------------------------------------------------------------
(define-public (psi-default-action-selector a-random-state)
"
  Returns a list of one of the most-important-weighted and satisfiable psi-rule
  or an empty list. A single psi-rule is returned so as help avoid mulitple
  actions of the same effect or type(aka semantic of the action) from being
  executed. If a satisfiable rule doesn't exist then the empty list is returned.

  a-random-state:
  - A random-state object used as a seed for choosing how multiple satisfiable
  psi-rules with the same weight are to be choosen.
"
    (define (choose-rules)
        ; NOTE: This check is required as ecan isn't being used continuesely.
        ; Remove `most-weighted-atoms` version once ecan is integrated.
        (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
            (most-weighted-atoms (psi-get-all-satisfiable-rules))
            (most-important-weighted-atoms (psi-get-all-satisfiable-rules))
        )
    )

    (let ((rules (choose-rules)))
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
