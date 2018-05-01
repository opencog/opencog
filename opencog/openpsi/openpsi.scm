; openpsi.scm
;
; Define (opencog openpsi) module.
;
; Copyright (C) 2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud


; --------------------------------------------------------------
(define-module (opencog openpsi)
  #:use-module (ice-9 optargs) ; For `define*`
  #:use-module (ice-9 threads) ; For `par-map`
  #:use-module (srfi srfi-1) ; For `drop-right`, `append-map`, etc.
  #:use-module (opencog)
  #:use-module (opencog exec)
  #:use-module (opencog logger)

  #:export (
    ; From action-selector.scm
    psi-set-action-selector! psi-action-selector
    psi-select-rules

    ; From rule.scm
    psi-get-rules psi-get-all-rules psi-get-all-actions
    psi-rule-alias
    psi-partition-rule-with-alias psi-related-goals
    psi-rule-satisfiability psi-get-satisfiable-rules
    psi-get-all-satisfiable-rules
    psi-get-all-weighted-satisfiable-rules psi-context-weight psi-action-weight
    psi-goal psi-goal? psi-rule-set-alias!

    ; From main.scm
    psi-running? psi-loop-count psi-run-continue? psi-step psi-run psi-halt
    psi-get-logger psi-component psi-set-dgv! psi-dgv
    psi-goal-value psi-set-gv! psi-urge psi-decrease-urge psi-increase-urge

    ; From utilities.scm
    psi-prefix-str psi-suffix-str

    ; C++ bindings from libopenpsi
    psi-action-executed?
    psi-add-category
    psi-add-to-category
    psi-categories
    psi-get-action
    psi-get-context
    psi-get-goal
    psi-imply
    psi-rule
    psi-rule?
    psi-satisfiable?
    )
)

(load-extension "libopenpsi" "opencog_openpsi_init")

; NOTE: The order of loading helps avoid warnings
(load-from-path "opencog/openpsi/utilities.scm")
(load-from-path "opencog/openpsi/rule.scm")
(load-from-path "opencog/openpsi/action-selector.scm")
(load-from-path "opencog/openpsi/main.scm")

; --------------------------------------------------------------
; Documentations for C++ bindings from libopenpsi
(set-procedure-property! psi-action-executed? 'documentation
"
  psi-action-executed? RULE

  Returns (stv 1 1) if the action was executed last time `psi-imply`
  was run, else it returns (stv 0 1).
"
)

(set-procedure-property! psi-get-action 'documentation
"
  psi-get-action RULE

  Get the action of the openpsi-rule RULE.  Returns the single
  atom that is the action.
"
)

(set-procedure-property! psi-get-context 'documentation
"
  psi-get-context RULE - Get the context of the openpsi-rule RULE.

  Returns a scheme list of all of the atoms that form the context.
"
)

(set-procedure-property! psi-get-goal 'documentation
"
  psi-get-goal RULE

  Get the goal of the openpsi-rule RULE.
"
)

(set-procedure-property! psi-imply 'documentation
"
  psi-imply RULE

  Execute the action of the RULE if its satisfiablity has been checked by
  using psi-satisfiable?. If it hasn't been checked it throws an error.
"
)

(set-procedure-property! psi-rule 'documentation
"
  psi-rule CONTEXT ACTION GOAL TV CATEGORY - create a psi-rule.

  Associate an action with a context such that, if the action is
  taken, then the goal will be satisfied. The structure of a rule
  is in the form of an `ImplicationLink`:

    (ImplicationLink TV
        (AndLink
            CONTEXT
            ACTION)
        GOAL)

  where:
  CONTEXT is a scheme list containing all of the terms that should
    be met for the ACTION to be taken. These are atoms that, when
    evaluated, should return a true or false TV.  The action is taken
    only if the boolean-AND of the return values is true.

  ACTION is an evaluatable atom, i.e. it should return a TV when
    evaluated by `cog-evaluate!`.

  GOAL is an atom that represents what goal is affected when an action
    is made in the specified context. If multiple goals are affected by
    the context and action then multiple psi-rules should be created.

  TV is the TruthValue assigned to the ImplicationLink. It should
    be a SimpleTruthValue.

  CATEGORY is a Node, representing the category that this rule is part of.
"
)

(set-procedure-property! psi-rule? 'documentation
"
  psi-rule? ATOM

  Returns `#t` or `#f`, depending on whether the passed argument
  is a valid openpsi-rule or not. A valid psi-rule is one that is created by
  `psi-rule` function.

  ATOM is the atom to check.
"
)

(set-procedure-property! psi-satisfiable? 'documentation
"
  psi-satisfiable? RULE - Return a TV indicating if the context of
  the RULE is satisfiable.

  Satisfaction is determined by evaluating the context part of the
  rule. If the context requires grounding in the atomspace, then
  this is performed. That is, if the context contains variables
  that require grounding, then this is performed. If the context is
  not groundable, then the rule is not satisfiable.

  The current implementation returns TRUE_TV if the context is
  satisfiable, else it returns FALSE_TV. A later design point
  is to have a weighted, probabilistic value to be returned.
"
)
