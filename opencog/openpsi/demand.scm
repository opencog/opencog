; Copyright (C) 2015 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (rnrs sorting)) ; For sorting demands by their values.

(use-modules (opencog) (opencog exec) (opencog rule-engine))

(load-from-path "openpsi/utilities.scm")

; TODO: Temporary hack for accesing `random-string` function.
(load-from-path "nlp/relex2logic/rule-helpers.scm")

; --------------------------------------------------------------
; Name of variables for common functions in this file-scope
; NOTE: Shouldn't be exported to prevent modification.
(define demand-var (VariableNode "Demand"))

(define demand-type
    (TypedVariableLink
        demand-var
        (TypeNode "ConceptNode"))
)

; --------------------------------------------------------------
(define-public (psi-demand-pattern)
"
  Returns an alist used to define the psi demand pattern. The key strings are,
  - 'var': its value is a list containing the VariableNodes and their type
           restrictions.
  - 'pat': its value is a DefinedPredicateNode that is associated with the
           demand pattern.
"
    (define z-alist (acons "var" (list demand-type) '()))
    (define dpn (DefinedPredicateNode
        (string-append (psi-prefix-str) "demand-pattern")))

    (DefineLink
        dpn
        (PresentLink
            (InheritanceLink
                demand-var
                (ConceptNode (string-append (psi-prefix-str) "Demand")))
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "default_value"))
                (ListLink
                    demand-var
                    (VariableNode "default_value")))))
    (acons "pat" dpn z-alist)
)

; --------------------------------------------------------------
(define (psi-clean-demand-gets set-link)
"
  Returns a list aftering removing the 'SetLink + 'ListLink + 'NumberNode
  from result of `cog-execute!` of 'GetLink wrapping psi-demand-pattern with or
  without additional clauses.

  set-link:
    - A 'SetLink that is a result of running `cog-execute!` of 'GetLink
      wrapping psi-demand-pattern.
"
; TODO: Make this generic for pattern gets
    (define (is-nn? x) (equal? (cog-type x) 'NumberNode))
    (remove! is-nn?
        (list-merge (par-map cog-outgoing-set (cog-outgoing-set set-link))))
)

; --------------------------------------------------------------
(define-public (psi-get-demands)
"
  Returns a list containing the ConceptNode that carry the demand-value. The
  strength of their stv is the demand value.
"
    (psi-clean-demand-gets (cog-execute!
        (GetLink
            (VariableList (assoc-ref (psi-demand-pattern) "var"))
            (assoc-ref (psi-demand-pattern) "pat"))))
)

; --------------------------------------------------------------
(define-public (psi-demand  demand-name default-value default-action)
"
  Define an OpenPsi demand, that will have a default behavior defined by the
  the action passed.

  demand-name:
  - The name of the demand that is created.

  default-value:
  - The initial demand-value. This is the strength of the demand's
    ConceptNode stv. The confidence of the stv is always 1.

  default-action:
  - The default action that modifies the demand-value.
"

    (let* ((demand-str (string-append (psi-prefix-str) demand-name))
           (demand-node (ConceptNode demand-str (stv default-value 1))))
        (begin
            (InheritanceLink
                demand-node
                (ConceptNode (string-append (psi-prefix-str) "Demand"))
            )

            ; XXX: Note sure this is needed. Possibly use is if one wants
            ; to measure how changes or define a default-action that updates
            ; based on the rate of change.
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "default_value"))
                (ListLink
                    demand-node
                    (NumberNode default-value)
                )
            )

            ; Add the default action.
            (psi-action
                (assoc-ref (psi-demand-pattern) "var")
                (list
                    (assoc-ref (psi-demand-pattern) "pat")
                    ; Filter out all other demands
                    (EqualLink demand-var demand-node)
                )
                default-action
                demand-node
                "Default")
            ; Each demand is also a rulebase
            (ure-define-rbs demand-node 1)
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-demand? atom)
"
  Checks whether an atom is the ConceptNode that satisfies the pattern used
  to define an OpenPsi demand. Returns True-TruthValue `(stv 1 1)` if it is
  and False-TruthValue `(stv 0 1)` if it isn't.

  atom:
  - The atom that is being checked to see if it is the Node that represents
    a demand type.
"
    (define demand-names (map cog-name (psi-get-demands)))
    (if (and (member (cog-name atom) demand-names)
             (equal? (cog-type atom) 'ConceptNode))
        (stv 1 1)
        (stv 0 1)
    )
)

; --------------------------------------------------------------
(define (psi-lowest-demand? atom)
"
  Returns #t if the atom passed is a demand that has the lowest demand-value.

  atom:
  - The atom that is being checked to see if it is the Node that represents
    a demand type, with a lowest demand-value.
"
    ; check if atom is a demand-node
    (if (equal? (stv 0 1) (psi-demand? atom))
        (error "Expected argument to be a demand-node, got: " atom))

    (let ((atom-strength (tv-mean (cog-tv atom)))
          (lowest-demand-value (car (list-sort < (delete-duplicates
              (map (lambda (x) (tv-mean (cog-tv x))) (psi-get-demands))))))
         )
         (if (<= atom-strength lowest-demand-value)
            (stv 1 1)
            (stv 0 1)
         )
    )
)

; --------------------------------------------------------------
(define-public (psi-select-goal gpn)
"
  Goal are defined as demands choosen for either increase or decrease in
  their demand values. The choice for being the current-goal is made by pattern
  matching over the demands by using the GroundedPredicateNode passed as a
  constraint.

  The GroundedPredicateNode passed should tag the demands for increase or
  decrease by evaluating the demands that satisfy the conditions set through
  its goal-selection function. The function should tag the goal representing
  atom (the demand-ConceptNode) for being increased or decreased.

  gpn:
    - GroundedPredicateNode that names an goal-selection function that will
      choose the demands. Its function should take a single demand-ConceptNode
      and return True-TruthValue `(stv 1 1)`  or False-TruthValue `(stv 0 1)`
      in addition to tagging the demand-ConceptNodes as one of the action-types.
      For example,

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Increase\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Decrease\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      The tags are necessary because, that is the means for signaling what type
      of actions should be taken, in effect it is the demand-goal. For example,
      if the action-on-demand is Increase, then only the actions of type
      Increase would be choosen.

"

    ; XXX: Should there be weight b/n the different demand-goals? For now a
    ; a random choice of demands is assumed. In subsequent steps. The
    ; demand-value could possibly be used for that.
    (define (get-demand) (psi-clean-demand-gets (cog-execute!
        (GetLink
            (VariableList (assoc-ref (psi-demand-pattern) "var"))
            (AndLink
                (assoc-ref (psi-demand-pattern) "pat")
                (EvaluationLink
                    gpn
                    (ListLink demand-var)))))))

    ; check arguments
    (if (not (equal? (cog-type gpn) 'GroundedPredicateNode))
        (error "Expected GroundedPredicateNode got: " gpn))

    ; TODO: 1. deal with multiple returns
    ;       2. check if the demadns have ben correctly tagged  instead add psi-register-goal-selector
    (get-demand)
)

; --------------------------------------------------------------
(define-public (psi-current-goal)
"
  Returns the demand-ConceptNode that has been choosen for action presently.
"

    (define (get-psi-goal) (cog-bind
        (BindLink
            (VariableList (assoc-ref (psi-demand-pattern) "var"))
            (AndLink
                (assoc-ref (psi-demand-pattern) "pat")
                (StateLink
                    (Node (string-append (psi-prefix-str) "action-on-demand"))
                    (ChoiceLink
                        (ListLink
                            (ConceptNode
                                (string-append (psi-prefix-str) "Decrease"))
                            demand-var)
                        (ListLink
                            (ConceptNode
                                (string-append (psi-prefix-str) "Increase"))
                            demand-var)))
                (EvaluationLink ; Act only if their is such a demand.
                    (GroundedPredicateNode "scm: psi-demand?")
                    (ListLink
                        demand-var)))
            demand-var)
    ))

    (let* ((set-link (get-psi-goal))
           (result (cog-outgoing-set set-link)))

          (cog-delete set-link)
          (if (null? result) result  (car result))
    )
)

; --------------------------------------------------------------
(define-public (psi-action-types)
"
  Returns a list of the default action types, that are used to describe how
  an action affects the demands it is associated with. Don't depend on the
  order of the list, it might change when effect-types are added/removed.
  The availabe action types are,

  Increase: increases the demand-value.
  Decrease: decreases the demand-value.
  Default: depends on the default-action associated with it. And is used
           to define how the demand-value should change independent of context.
           Remember that the effect of this rule could be either increasing or
           decreasing of the demand-value as well, but wouldn't be known as
           such.
"
    ; NOTE: Update psi-update-asp and psi-get-actions-all
    ; when adding other effect types.
    (list "Increase" "Decrease" "Default")
)

; --------------------------------------------------------------
(define-public (psi-action vars context action demand-node effect-type)
"
  It associates an action and context in which the action has to be taken
  to an OpenPsi-demand. The action is also a member rule of the demand it
  affects, with a weight of one. The returned node defines/aliases the
  BindLink structured as,
    (BindLink
        (VariableList (vars))
        (AndLink
            (context)
            (clauses for differentiating the action))
        (action))

  A single action could only have one effect-type, thus changing the
  effect-type will not have any effect, if the action-rule has
  already been defined in the atomspace with a different effect-type.

  vars:
    - A list containing the VariableNodes, and their type restrictions, that
      are part of the context. If there is no type restrictions then pass empty
      list.

  context:
    - A list containing the terms/clauses that should be met for this action
      to be taken. Be careful on how you use Variable naming in the context

  action:
    - The Implicand of the rule. It should be an atom that uses the groundings
      of the context to do something.

  demand-node:
  - The ConceptNode representing the demand.

  effect-type:
    - A string that describes the effect the particualr action would have on
      the demand value. See `(psi-action-types)` for available options.

"
    (define rule-name-prefix (string-append (cog-name demand-node) "-rule-"))
    (define rule-name (string-append rule-name-prefix (random-string 5)))

    (define (rule)
        ; Is function to avoid  insertion into the atomspace if argument check
        ; fails.
        (BindLink
            ; An empty VariableList prevents matchs.
            (if (equal? '() vars)
                '()
                (VariableList vars)
            )
            (AndLink
                context
                (EvaluationLink ; Act only if their is such a demand.
                    (GroundedPredicateNode "scm: psi-demand?")
                    (ListLink demand-node)))
            action))

    (define (create-psi-action)
        (let ((alias (ure-add-rule demand-node rule-name (rule) 1)))
            (InheritanceLink
                alias
                (ConceptNode "opencog: action"))

            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) effect-type))
                (ListLink
                    alias
                    demand-node))
            alias
        ))

    ; Check arguments
    (if (not (list? vars))
        (error "Expected first argument to be a list, got: " vars))
    (if (not (list? context))
        (error "Expected second argument to be a list, got: " context))
    (if (not (cog-atom? action))
        (error "Expected third argument to be an atom, got: " action))
    (if (not (member effect-type (psi-action-types)))
        (error (string-append "Expected fourth argument to be one of the "
            "action types listed when running `(psi-action-types)`, got: ")
            effect-type))

    ; Check if the rule has already been defined as a member of
    ; TODO: this needs improvement not exaustive enough, it isn't considering
    ;       other differentiating graphs.
    (let ((node (cog-chase-link 'DefineLink 'Node (rule))))
        (cond ((and (= 1 (length node))
                    (string-prefix? rule-name-prefix (cog-name (car node))))
                     node)
              ((= 0 (length node))
               (create-psi-action))
              (else ; cleanup TODO: Make exaustive
                  (cog-delete (rule))
                  (error "The rule has been defined multiple times"))
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-get-actions demand-node effect-type)
"
  Returns a list containing the 'Node atom-type atoms that name the action-rules
  for the given demand-node.

  demand-node:
    - A ConceptNode that represents a demand.

  effect-type:
    - A string that describes the effect the particualr action would have on
      the demand value. See `(psi-action-types)` for available options.
"
    ; Check arguments
    (if (not (member effect-type (psi-action-types)))
        (error (string-append "Expected fourth argument to be one of the "
            "action types listed when running `(psi-action-types)`, got: ")
            effect-type))

    (cog-outgoing-set (cog-execute!
        (GetLink
             (TypedVariableLink
                 (VariableNode "x")
                 (TypeNode "DefinedSchemaNode"))
             (AndLink
                 (EvaluationLink
                     (PredicateNode
                         (string-append (psi-prefix-str) effect-type))
                     (ListLink
                        (VariableNode "x")
                        demand-node))
                 (InheritanceLink
                     (VariableNode "x")
                     (ConceptNode "opencog: action"))))
    ))
)

; --------------------------------------------------------------
(define-public (psi-get-actions-all demand-node)
"
  Returns a list containing the 'Node atom-type atoms that name the action-rules
  for the given demand-node, with the exeception of the Default action.

  demand-node:
    - A ConceptNode that represents a demand.
"
    ; NOTE: Not using (ure-rbs-rules demand-node) so as to not include 'Default'
    ; action-types, as this function is used by psi-update-asp.
    (append
        (psi-get-actions demand-node "Increase")
        (psi-get-actions demand-node "Decrease")
    )
)

; --------------------------------------------------------------
(define-public (psi-get-actions-default)
"
  Returns the default actions for all the defined demands
"
    (append-map (lambda (x) (psi-get-actions x "Default"))
        (psi-get-demands))
)

; --------------------------------------------------------------
(define-public (psi-select-actions demand-node gpn)
"
  Select the actions that should be added to the active-schema-pool depending
  on the present goal, by using the plan choosen by the GroundedPredicateNode.

  demand-node:
    - A ConceptNode that represents a demand

  gpn:
   - GroundedPredicateNode that refers to a function that checks the actions
     for constraints.
"
    ;TODO: I think the planner is kind of a behavior tree genrator (assuming
    ; there is no change of a preset plan) .URE's random selection policy
    ; isn't being used now thus each plan is in effect a single action choosen,
    ; this has to be improved but is good for starters.

    ; Check arguments
    (if (not (equal? (cog-type gpn) 'GroundedPredicateNode))
        (error "Expected GroundedPredicateNode got: " gpn))

    (cog-outgoing-set (cog-execute!
        (GetLink
             (TypedVariableLink
                 (VariableNode "x")
                 (DefinedSchemaNode "Node"))
             (AndLink
                 (EvaluationLink
                     gpn
                     (ListLink  (VariableNode "x")))
                 (MemberLink
                     (VariableNode "x")
                     demand-node)
                 (InheritanceLink
                     (VariableNode "x")
                     (ConceptNode "opencog: action"))))
    ))
)

; --------------------------------------------------------------
(define-public (psi-current-effect-type)
"
  This returns a string of the type of effect that the current-goal have.
"
   (define (get-psi-action-type) (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "effect-type")
                (TypeNode "ConceptNode"))
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ListLink
                    (VariableNode "effect-type")
                    (psi-current-goal))))
    ))

    (let* ((set-link (get-psi-action-type))
          (result (cog-outgoing-set set-link)))

          (cog-delete set-link)

          (if (null? result)
              "Default"
              (psi-suffix-str (cog-name (car result)))
          )
    )
)

; --------------------------------------------------------------
(define-public (psi-effect-maximize rate)
"
  Returns an ExecutionOutputLink that increases the demand-value. This is
  the effect of the action. It has a increasing effect-type.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step.
"

    (ExecutionOutputLink
        (GroundedSchemaNode "scm: psi-value-maximize")
        (ListLink
            demand-var
            (NumberNode rate)))
)

(define-public (psi-value-maximize demand rate-node)
    (let ((strength (tv-mean (cog-tv  demand)))
          (conf (tv-conf (cog-tv demand)))
          (rate (/ (string->number (cog-name rate-node)) 100)))
        (cog-set-tv! demand (stv (+ strength (* strength rate)) conf))
    )
)

(define-public (psi-action-maximize demand-node rate)
"
  Creates an action with the effect of increasing the demand-value for the
  given demand.

  demand-node:
  - The ConceptNode representing the demand.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step. If an action with the same rate of has been defined for the
    given demand a new action isn't created, but the old one returned.
"

    (psi-action
        (assoc-ref (psi-demand-pattern) "var")
        (list
            (assoc-ref (psi-demand-pattern) "pat")
            ; Filter out all other demands
            (EqualLink demand-var demand-node)
        )
        (psi-effect-maximize rate)
        demand-node
        "Increase")
)

; --------------------------------------------------------------
(define-public (psi-effect-minimize rate)
"
  Returns an ExecutionOutputLink(the action) that decreases the demand-value.
  It has a decreasing effect-type.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step.
"

    (ExecutionOutputLink
        (GroundedSchemaNode "scm: psi-value-minimize")
        (ListLink
            demand-var
            (NumberNode rate)))
)

(define-public (psi-value-minimize demand rate-node)
    (let ((strength (tv-mean (cog-tv  demand)))
          (conf (tv-conf (cog-tv demand)))
          (rate (/ (string->number (cog-name rate-node)) 100)))
        (cog-set-tv! demand (stv (- strength (* strength rate)) conf))
    )
)

(define-public (psi-action-minimize demand-node rate)
"
  Creates an action with the effect of decreasing the demand-value for the
  given demand.

  demand-node:
  - The ConceptNode representing the demand.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step. If an action with the same rate of has been defined for the
    given demand a new action isn't created, but the old one returned.
"

    (psi-action
        (assoc-ref (psi-demand-pattern) "var")
        (list
            (assoc-ref (psi-demand-pattern) "pat")
            ; Filter out all other demands
            (EqualLink demand-var demand-node)
        )
        (psi-effect-minimize rate)
        demand-node
        "Decrease")
)

; --------------------------------------------------------------
(define-public (psi-effect-keep-range min-value max-value rate)
"
  Returns an ExecutionOutputLink(the action) that tries to maintain the value
  within the specified range. If the demand-value is within range then it does
  nothing.

  It is mainily to be used as a default effect-type, as it can increase or
  decrease the demand value.

  min-value:
  - A number for the minimum acceptable demand-value.

  max-value:
  - A number for the maximum acceptable demand-value.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step, should it pass the boundaries.
"
    ; Check arguments
    (if (or (> min-value max-value) (>= 0 min-value) (<= 1 max-value))
       (error "Expected the range to be a subset of (0, 1) interval, got: "
              (string-append "(" (number->string min-value) ", "
                  (number->string max-value) ")" ))
    )
    (if (>= 0 rate)
       (error "Expected the percentage of change for the demand value "
              "to be > 0, got: " rate))

    (ExecutionOutputLink
        (GroundedSchemaNode "scm: psi-value-range")
        (ListLink
            demand-var
            (NumberNode min-value)
            (NumberNode max-value)
            (NumberNode rate)))
)

; --------------------------------------------------------------
(define-public (psi-value-range demand min-node max-node rate-node)
"
  Increases or decreases the strength of the demand depending on whether it is
  in between the range specified. The range is taken as an open-interval, that
  must be a subset of (0, 1) interval.

  demand:
  - The ConceptNode that represents the demand.

  min-node:
  - A NumberNode for the lower boundary of the range.

  max-node:
  - A NumberNode for the upper boundary of the range.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value be updated
    with, should it pass the boundaries. Must be greater than zero.
"

    (let ((mean (tv-mean (cog-tv  demand)))
          (min-value (string->number (cog-name min-node)))
          (max-value (string->number (cog-name max-node)))
          (rate (/ (string->number (cog-name rate-node)) 100)))

         ; Maximize or minmize
         (cond ((strength > max-value) (psi-value-minimize demand rate-node))
               ((strength < min-value) (psi-value-maximize demand rate-node))
         )
    )
)
