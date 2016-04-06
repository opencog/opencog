; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog exec))

(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define (psi-goal-selector-pattern)
"
  This returns the StateLink that is used for specifying the goal selecting
  evaluatable term.

  A StateLink is used instead of an InheritanceLink because there could only
  be one active goal-selector at a time eventhough there could be multiple
  possible goal-selectors. And this enables dynamically changing the
  goal-selector through learning.
"
    (StateLink
        (ConceptNode (string-append (psi-prefix-str) "goal-selector"))
        (VariableNode "$dpn")
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-set! dpn)
"
  Sets the given DefinedPredicateNode to be used for selecting goals.

  dpn:
  - The DefinedPredicateNode that represents the evaluatable-term used for
    selecting demand to be a goal.
"
    ; Check arguments
    (if (not (equal? (cog-type dpn) 'DefinedPredicateNode))
        (error "Expected DefinedPredicateNode got: " dpn))
    (cog-execute!
        (PutLink
            (psi-goal-selector-pattern)
            dpn)
    )
)

; --------------------------------------------------------------
(define (psi-add-goal-selector eval-term effect-type name)
"
  Returns the DefinedPredicateNode that represents the evaluatable term
  after defining it as an opencog goal-selector.

  eval-term:
  - An evaluatable term.

  effect-type:
  - A string that describes the effect the particualr action would have on
    the demand value. See `(psi-action-types)` for available options.

  name:
  -  A string for naming the goal selector. `OpenPsi: goal-selector-`
     will be prefixed to the name.
"
    ; Check arguments
    (if (not (string? name))
        (error "Expected third argument to be a string, got: " name))
    (if (not (member effect-type (psi-action-types)))
        (error (string-append "Expected second argument to be one of the "
            "action types listed when running `(psi-action-types)`, got: ")
            effect-type))

    ; TODO: Add checks to ensure the eval-term argument is actually evaluatable
    (let* ((z-name (string-append (psi-prefix-str) "goal-selector-" name))
           (goal-selector-dpn (DefinedPredicateNode z-name)))
        ; This must be first so as to check if a DefinedPredicateNode of the
        ; same name is already defined.
        (DefineLink goal-selector-dpn eval-term)

        (EvaluationLink
            (PredicateNode "selects-for-effect-type")
            (ListLink
                goal-selector-dpn
                (ConceptNode (string-append (psi-prefix-str) effect-type))))

        goal-selector-dpn
    )
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

          (cog-extract set-link)
          (if (null? result) result  (car result))
    )
)

; --------------------------------------------------------------
(define-public (psi-current-effect-type)
"
  This returns a string of the type of effect that the current-goal has.
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

          (cog-extract set-link)

          (if (null? result)
              "Default"
              (psi-suffix-str (cog-name (car result)))
          )
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-effect-type dpn)
"
  Returns a node representing the effect type of the given goal-selector.

  dpn:
  - The DefinedPredicateNode that represents the evaluatable-term used for
    selecting demand to be a goal.
"
    ; Check arguments
    (if (not (equal? (cog-type dpn) 'DefinedPredicateNode))
        (error "Expected DefinedPredicateNode got: " dpn))

    ; See https://github.com/opencog/atomspace/issues/646 why this doesn't
    ; work
    ;(GetLink
    ;    (TypedVariableLink
    ;        (VariableNode "$effect")
    ;        (TypeNode "ConceptNode"))
    ;        (EvaluationLink
    ;            (PredicateNode "selects-for-effect-type")
    ;            (ListLink
    ;                (QuoteLink dpn)
    ;                (VariableNode "$effect")))
    ;)

    ; NOTE: Assuming the dpn isn't part of any other similar pattern.
    (car (cog-chase-link 'ListLink 'ConceptNode dpn))
)

; --------------------------------------------------------------
;  Goal selectors
; --------------------------------------------------------------
(define (psi-select-random-goal)
"
  Returns the StateLink representing the goal. Goal are defined as demands
  choosen for either increase or decrease in their demand values. For example,

   (StateLink
       (Node (string-append (psi-prefix-str) \"action-on-demand\"))
       (ListLink
           (ConceptNode (string-append (psi-prefix-str) \"Increase\"))
           (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))
  or

   (StateLink
       (Node (string-append (psi-prefix-str) \"action-on-demand\"))
       (ListLink
           (ConceptNode (string-append (psi-prefix-str) \"Decrease\"))
           (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

  The StateLink(aka goal) is the means for signaling what type of actions
  should be selected.
"
    (define (set-goal a-demand effect-type)
       (StateLink
           (Node (string-append (psi-prefix-str) "action-on-demand"))
           (ListLink effect-type a-demand))
    )

    (define (get-goal-selector)
        (let ((goal-selector (cog-outgoing-set (cog-execute!
                    (GetLink (psi-goal-selector-pattern))))))
            (if (null? goal-selector)
                (error "A goal-selector hasn't been set for OpenPsi.")
                (car goal-selector)
            )
        )
    )

    (let* ((goal-selector (get-goal-selector))
           (demands (psi-get-demands goal-selector))
           (effect (psi-goal-selector-effect-type goal-selector)))

       ; If there are no demands that satisfy the condition then choose one
        (if (null? (cog-outgoing-set demands))
            (set-goal
                (cog-execute! (RandomChoiceLink (psi-get-demands-all)))
                (ConceptNode (string-append (psi-prefix-str) "Default")))
            (set-goal (cog-execute! (RandomChoiceLink demands)) effect)
        )
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-maximize threshold)
"
  Sets the goal by randomly selecting a demand for maximization, should its
  demand-value be below the given threshold.

  threshold:
  - The boundary of the demand-value below which a demand will be chosen.
"
    (psi-add-goal-selector
        (psi-demand-value-term< threshold) "Increase" "maximize")
)
