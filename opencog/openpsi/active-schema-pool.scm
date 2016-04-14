; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (srfi srfi-1)) ; For set-difference

(use-modules (opencog) (opencog exec) (opencog rule-engine))

(load-from-path "openpsi/action-selector.scm")
(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define-public (psi-asp)
"
  Returns the ConceptNode that represents the active-schema-pool(asp). The asp
  is a set of all the psi-rules.
"
    (ConceptNode (string-append (psi-prefix-str) "asp"))
)

; --------------------------------------------------------------
(define-public (psi-step)
"
  The main function that defines the steps to be taken in every cycle.
"
    (let* ((rules (psi-select-rules)))
        (map (lambda (x)
                (cog-execute! (psi-get-action x))
                (cog-evaluate! (psi-get-goal x)))
            rules)
    )
)

; --------------------------------------------------------------
(define (psi-rule context action demand-goal stv)
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

  stv:
  - This is the stv of the ImplicationLink.
"
    (define (implication)
        (ImplicationLink stv (AndLink context action) demand-goal))

    ; Check arguments
    (if (not (list? context))
        (error "Expected first argument to psi-rule to be a list, got: "
            context))
    (if (not (cog-atom? action))
        (error "Expected second argument to psi-rule be an atom, got: " action))
    (if (not (cog-tv? stv))
        (error "Expected fourth argument to psi-rule to be a stv, got: "
            stv))

    ; These memberships are needed for making filtering and searching simpler..
    ; If GlobNode had worked with GetLink at the time of coding this ,
    ; that might have been; better,(or not as it might need as much chasing)
    (MemberLink
        action
        (ConceptNode (string-append (psi-prefix-str) "action")))

    (MemberLink (implication) (psi-asp))

    (implication)
)

(define (psi-get-all-actions) ; get openpsi actions
    (cog-outgoing-set (cog-execute! (GetLink
        (MemberLink (VariableNode "x")
        (ConceptNode (string-append (psi-prefix-str) "action")))))))

(define (psi-action? x)
    (if (member x (psi-get-all-actions)) #t #f))

(define (psi-get-all-rules) ; get all openpsi rules
    (cog-chase-link 'MemberLink 'ImplicationLink (psi-asp)))

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

(define (psi-default-action-selector)
"
  Get all psi-rules that are satisfiable
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-all-rules))
)

(define (psi-select-rules)
"
  Returns a list of psi-rules that are satisfiable by using the action-selector
  you defined or the default-action-selector predefined if you haven't defined
  a different action-selector.
"
    (let ((dsn (psi-get-action-selector)))
        (if (null? dsn)
            (psi-default-action-selector)
            (cog-execute! dsn)
        )
    )
)
