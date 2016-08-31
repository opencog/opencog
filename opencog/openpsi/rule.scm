; Copyright (C) 2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (ice-9 optargs)) ; For `define*-public`
(use-modules (srfi srfi-1)) ; For `append-map`
(use-modules (opencog) (opencog query))

(load "demand.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define psi-action (ConceptNode (string-append psi-prefix-str "action")))
(define psi-rule-name-predicate-node
    (PredicateNode (string-append psi-prefix-str "rule_name")))

; --------------------------------------------------------------
(define*-public (psi-rule-nocheck context action goal a-stv demand
     #:optional name)
"
  psi-rule-nocheck -- same as psi-rule, but no checking
"
    (let ((implication (Implication a-stv (And context action) goal)))

    ; These memberships are needed for making filtering and searching simpler..
    ; If GlobNode had worked with GetLink at the time of coding this,
    ; that might have been; better, (or not as it might need as much chasing)
    ; TODO: Remove this when ExecutionLinks are used as that can be used
    ; for filtering.
        (MemberLink action psi-action)

    ; This memberLink is added so as to minimize the tree walking needed to get
    ; from a goal to a psi-rule. For example, if the goal is a DefinedPredicate.
        (MemberLink implication demand)

        ; Give name this is used for control/feedback purposes
        ; Why EvaluationLink? B/c we can't use DefineLink. Maybe when protoatom
        ; are ready that could be used????
        (if name
            (EvaluationLink
                psi-rule-name-predicate-node
                (ListLink
                    implication
                    (ConceptNode (string-append psi-prefix-str name))))
        )

        implication
    )
)

; --------------------------------------------------------------
(define*-public (psi-rule context action goal a-stv demand  #:optional name)
"
  psi-rule CONTEXT ACTION GOAL TV DEMAND - create a psi-rule.

  Associate an action with a context such that, if the action is
  taken, then the goal will be satisfied.  The structure of a rule
  is in the form of an `ImplicationLink`,

    (ImplicationLink TV
        (AndLink
            CONTEXT
            ACTION)
        GOAL)

  where:
  CONTEXT is a scheme list containing all of the terms that should
    be met for the ACTION to be taken. These are atoms that, when
    evaluated, should result in a true or false TV.

  ACTION is an evaluatable atom, i.e. returns a TV when evaluated by
    `cog-evaluate!`.  It should return a true or false TV.

  GOAL is an evaluatable atom, i.e. returns a TV when evaluated by
    `cog-evaluate!`.  The returned TV is used as a formula to rank
    how this rule affects the demands.

  TV is the TruthValue assigned to the ImplicationLink. It should
    be a SimpleTruthValue.

  DEMAND is a Node, representing the demand that this rule affects.
"
; TODO: aliases shouldn't be used to create a subset, as is done in chatbot-psi
; It should be done at the demand level. The purpose of an aliase should only
; be for controlling a SINGLE rule during testing/demoing, and not a set
; of rules.
    (define func-name "psi-rule") ; For use in error reporting

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
    (if (not (psi-demand? demand))
        (error (string-append "In procedure " func-name ", expected fifth "
            "argument to be a node representing a demand, got:") demand))

    (psi-rule-nocheck context action goal a-stv demand name)
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

XXX FIXME -- this is painfully slow --- multiple minutes when
there are 100K rules!
"
    (fold append '()
        (par-map (lambda (x) (cog-chase-link 'MemberLink 'ImplicationLink x))
            (psi-get-all-demands)))
)

; --------------------------------------------------------------
(define-public (psi-rule? atom)
"
  Returns `#t` or `#f` depending on whether the passed argument
  is a valid psi-rule or not. An ImplicationLink that is a member
  of the demand sets is a psi-rule.

  atom:
  - An atom passed for checking.
"
    (let ((candidates (cog-chase-link 'MemberLink 'ConceptNode atom)))

        ; A filter is used to account for empty list as well as
        ; cog-chase-link returning multiple results, just in case.
        (not (null?
            (filter psi-demand? candidates)))
    )
)

; --------------------------------------------------------------
(define-public (psi-get-all-actions)
"
  Returns a list of all openpsi actions.
"
    (append
        (cog-chase-link 'MemberLink 'ExecutionOutputLink psi-action)
        (cog-chase-link 'MemberLink 'DefinedSchemaNode psi-action))
)

; --------------------------------------------------------------
(define-public (psi-action? ATOM)
"
  Check if ATOM is an action and return `#t`, if it is, and `#f`
  otherwise. An atom is an action if it a member of the set
  represented by (ConceptNode \"OpenPsi: action\").
"
    (let ((candidates (cog-chase-link 'MemberLink 'ConceptNode ATOM)))

        ; A filter is used to account for empty list as well as
        ; cog-chase-link returning multiple results, just in case.
        (not (null?
            (filter (lambda (x) (equal? x psi-action)) candidates)))
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
(define-public (psi-rule-alias psi-rule)
"
  Returns a list with the node that aliases the psi-rule if it has a an alias,
  or Returns null if it doesn't.

  psi-rule:
  - An ImplicationLink whose weight is going to be modified.
"
    (cog-outgoing-set (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "rule-alias")
                (TypeNode "ConceptNode"))
            (EvaluationLink
                psi-rule-name-predicate-node
                (ListLink
                    (QuoteLink psi-rule)
                    (VariableNode "rule-alias"))))))
)

; --------------------------------------------------------------
(define-public (psi-partition-rule-with-alias rule-alias psi-rule-list)
"
  Returns multiple values of lists with rules which have alias equaling
  `rule-alias` as the first value and those that don't as second value.

  rule-alias:
  - string used for a psi-rule alias.

  psi-rule-list:
  - a list with psi-rules.
"
    (partition
        (lambda (psi-rule)
            (equal?
                (Concept (string-append psi-prefix-str rule-alias))
                (car (psi-rule-alias psi-rule))))
        psi-rule-list)
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
  Check if the rule is satisfiable and return TRUE_TV or FALSE_TV. A rule is
  satisfiable when it's context is fully groundable. The idea is only
  valid when ranking of context grounding isn't consiedered. This is being
  replaced.

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
(define-public (psi-get-weighted-satisfiable-rules demand-node)
"
  Returns a list of all the psi-rules that are satisfiable and
  have a non-zero strength
"
    (filter
        (lambda (x) (and (> (cog-stv-strength x) 0)
            (equal? (stv 1 1) (psi-satisfiable? x))))
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
(define-public (psi-get-all-weighted-satisfiable-rules)
"
  Returns a list of all the psi-rules that are satisfiable and
  have a non-zero strength
"
    (filter
        (lambda (x) (and (> (cog-stv-strength x) 0)
            (equal? (stv 1 1) (psi-satisfiable? x))))
        (psi-get-all-rules))
)
