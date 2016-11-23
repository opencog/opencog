;
; rule.scm
; Utilities for defining and working with OpenPsi rules.
;
; Copyright (C) 2016 OpenCog Foundation
;
; Design Notes:
; The design below uses AndLinks instead of SequentialAndLinks to hold
; the context and the action. Recall that the AndLink is an unordered
; link, and so the context and action is stored in arbitrary order.
; This makes several of the methods inefficeint, as it requires this
; bag to be searched for the desired bits. If instead, an ordered link
; was used, and the action was made first, it could be found very
; quickly. FIXME -- this should be fixed someday.
;
; Aliases: Rules can be given a "name", called an "alias", below.
; Certain parts of the design assume that a rule has only one name,
; but other parts of the code pass around lists of names. This can
; lead to crazy bugs, due to a lack of checking or enforcement of a
; one-name-per-rule policy.  Also -- each rule should, in principle
; have a unique name; however, the current chatbot uses the same name
; for all chat rules.


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
  psi-rule-nocheck -- same as psi-rule, but does not check the arguments
  for proper structure.
"

    ; Note that the AndLink is an unordered link -- so the context
    ; and the action will appear in an arbitrary order.  It would
    ; almost surely be better to use a SequentialAndLink here;
    ; this would use less resources and have lower complexity.
    (let ((implication (Implication a-stv (AndLink context action) goal)))

        ; The membership below is used to simplify filtering and searching.
        ; Other alternative designs are possible.
        ; TODO: Remove this, when ExecutionLinks are used, as that can be used
        ; for filtering. (?? Huh? Please explain...)
        (MemberLink action psi-action)

        ; This MemberLink is used to make it easy to find rules that
        ; fulfil demands. (and also to find goals that meet demands).
        (MemberLink implication demand)

        ; If a name is given, its used for control/feedback purposes.
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
  psi-rule CONTEXT ACTION GOAL TV DEMAND [NAME] - create a psi-rule.

  Associate an action with a context such that, if the action is
  taken, then the goal will be satisfied.  The structure of a rule
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
    evaluated by `cog-evaluate!`.  The return value is currently
    ignored.

  GOAL is an evaluatable atom, i.e. returns a TV when evaluated by
    `cog-evaluate!`.  The returned TV is used as a formula to rank
    how this rule affects the demands.

  TV is the TruthValue assigned to the ImplicationLink. It should
    be a SimpleTruthValue.

  DEMAND is a Node, representing the demand that this rule affects.

  NAME is an optional argument; if it is provided, then it should be
    a string that will be associated with the rule.
"
; TODO: aliases shouldn't be used to create a subset, as is done in
; chatbot-psi.  Subsets should be created at the demand level. The
; purpose of an alias is only for controlling a SINGLE rule during 
; testing/demoing, and not a set of rules.
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
  psi-rule? ATOM

  Returns `#t` or `#f`, depending on whether the passed argument
  is a valid psi-rule or not. A psi-rule is any ImplicationLink that
  is a member of a demand set.

  ATOM is the atom to check.
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

XXX FIXME this is borken, and does not work.
actions are EvaluationLinks, not schemas or ExecutionOutputLinks.
"
    ;(append
    ;    (cog-chase-link 'MemberLink 'ExecutionOutputLink psi-action)
    ;    (cog-chase-link 'MemberLink 'DefinedSchemaNode psi-action))
    (list)
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
    (cog-outgoing-set (cog-outgoing-atom impl-link 0))
)

; --------------------------------------------------------------
(define-public (psi-get-context rule)
"
  psi-get-context RULE

  Get the context of the openpsi-rule RULE.  Returns a scheme
  list of all of the atoms that form the context.
"
    ;; FIXME: This is not an efficient way to get the context.
    ;; If this needs to be called a lot, it should be fixed.
    (remove psi-action? (get-c&a rule))
)

; --------------------------------------------------------------
(define-public (psi-get-action rule)
"
  psi-get-action RULE

  Get the action of the openpsi-rule RULE.  Returns the single
  atom that is the action.
"
    ;; FIXME: This is not an efficient way to get the action.
    ;; If this needs to be called a lot, it should be fixed.
    (car (filter psi-action? (get-c&a rule)))
)

; --------------------------------------------------------------
(define-public (psi-get-goal rule)
"
  psi-get-goal RULE

  Get the goal of the openpsi-rule RULE.
"
    ; NOTE: Why this function? -> For consisentency and to accomodate future
    ; changes
    (cadr (cog-outgoing-set rule))
)

; --------------------------------------------------------------
(define-public (psi-rule-alias psi-rule)
"
  psi-rule-alias RULE

  Returns a list of the aliases associated with the psi-rule RULE.
  An alias is just a name that is associated with the rule; it
  just provides an easy way to refer to a rule.
"
    ;; Using a GetLink here is quite inefficient; this would run much
    ;; faster if cog-chase-link was used instead. FIXME.
    (cog-outgoing-set (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "rule-alias")
                (TypeNode "ConceptNode"))
            (EvaluationLink
                psi-rule-name-predicate-node
                (ListLink
                    (QuoteLink psi-rule)  ;; ?? why is a Quote needed here?
                    (VariableNode "rule-alias"))))))
)

; --------------------------------------------------------------
(define-public (psi-partition-rule-with-alias rule-alias psi-rule-list)
"
  psi-partition-rule-with-alias ALIAS RULE-LIST

  Partitions the RULE-LIST into two lists: the first list will
  contain all rules that are tagged with the name ALIAS; the second
  list are all the other rules.  ALIAS should be a string.
"
    (define cali (Concept (string-append psi-prefix-str rule-alias)))
    (partition
        (lambda (rule) (equal? cali (car (psi-rule-alias rule))))
        psi-rule-list)
)

; --------------------------------------------------------------
(define-public (psi-related-goals action)
"
  psi-related-goals ACTION

  Return a list of all of the goals that are associated with the ACTION.
  A goal is associated with an action whenever the goal appears in
  a psi-rule with that action in it.
"
    ;; XXX this is not an efficient way of searching for goals.  If
    ;; this method is used a lot, we should convert to SequentialAnd,
    ;; and search for the goals directly.
    (let* ((and-links (cog-filter 'AndLink (cog-incoming-set action)))
           (rules (filter psi-rule? (append-map cog-incoming-set and-links))))
           (delete-duplicates (map psi-get-goal rules))
    )
)

; --------------------------------------------------------------
(define-public (psi-satisfiable? rule)
"
  psi-satisfiable? RULE

  Check if the RULE is satisfiable; return TRUE_TV if it is, else return
  FALSE_TV. A rule is satisfiable when it's context contains variables,
  and that context can be grounded in the atompace (i.e. if the context
  is satisfiable in using a SatisfactionLink.) 

  The idea is only valid when ranking of context grounding isn't considered.
  This is being replaced.  Huh??? What does this mean?
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
  psi-get-satisfiable-rules DEMAND

  Returns a list of all of the psi-rules associated with the DEMAND
  that are also satisfiable.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define-public (psi-get-weighted-satisfiable-rules demand-node)
"
  psi-get-weighted-satisfiable-rules DEMAND

  Returns a list of all the psi-rules that are satisfiable and
  have a non-zero strength.
"
    (filter
        (lambda (x) (and (> (cog-stv-strength x) 0)
            (equal? (stv 1 1) (psi-satisfiable? x))))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define-public (psi-get-all-satisfiable-rules)
"
  psi-get-all-satisfiable-rules

  Returns a list of all the psi-rules that are satisfiable.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-all-rules))
)

; --------------------------------------------------------------
(define-public (psi-get-all-weighted-satisfiable-rules)
"
  psi-get-all-weighted-satisfiable-rules

  Returns a list of all the psi-rules that are satisfiable and
  have a non-zero strength.
"
    (filter
        (lambda (x) (and (> (cog-stv-strength x) 0)
            (equal? (stv 1 1) (psi-satisfiable? x))))
        (psi-get-all-rules))
)
