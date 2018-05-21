;
; control.scm
;
; Copyright (C) 2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud
;
; Design Notes:
; Weights are associated to rules using a StateLink.  However, the
; design calls for the use of an AtTimeLink instead. Why is this needed?
; If it really is needed, then this neeeds to be fixed.

(use-modules (ice-9 receive))

(use-modules (opencog) (opencog exec) (opencog query))

(load "demand.scm")
(load "rule.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define (psi-set-updater! component-node updater)
"
  psi-set-updater! COMPONENT UPDATER

  An UPDATER is an evaluatable atom that, when evaluated, updates the
  values for a given COMPONENT.

  The COMPONENT is a component node that the updater is being added to.
"
    (psi-set-func! updater "#t" component-node "updater")
)

; --------------------------------------------------------------
(define (psi-get-updater component-node)
"
  psi-get-updater COMPONENT

  Returns the updater atom for the given COMPONENT, which when evaluated
  will update the weight of psi-rules. Null is returned if there is no
  updater for COMPONENT.
"
    (psi-func component-node "updater")
)

; --------------------------------------------------------------
; This demand used to tag a psi-rule that is being manually controlled.
; It controls the weight of other demands through the updater associated with
; it. It shouldn't be skipped.
; NOTE: This is a hack b/c once the weight of the rules is separated into atom
; or protoatom wrapped in a StateLink then (or somehting like that) then it
; would be easier to update.
(define psi-controller-demand (psi-demand "controller"))

; --------------------------------------------------------------
; To indicate whether the weights of the psi-controlled-rules
; are currently being updated or not, so as to avoid any sync problem.
; Could also be used to make sure that there is only one process
; updating the weights at a time.

; The states of the psi-controller
(define psi-controller (Anchor
    (string-append psi-prefix-str "psi-controller")))

(define psi-controller-idle
    (Concept (string-append psi-prefix-str "psi-controller-idle")))

(define psi-controller-busy
    (Concept (string-append psi-prefix-str "psi-controller-busy")))

(State psi-controller psi-controller-idle)

; -----
; For locking and releasing the controller

(define (psi-controller-occupy)
    (while (equal? psi-controller-busy
            (gar (cog-execute! (Get (State psi-controller (Variable "$x"))))))
        (sleep 0.5)
    )

    (State psi-controller psi-controller-busy)
)

(define (psi-controller-release)
    (State psi-controller psi-controller-idle)
)

; --------------------------------------------------------------
; FIXME -- can we have a shorter/better name for this method?
;
(define (psi-rule-set-atomese-weight psi-rule weight)
"
  psi-rule-set-atomese-weight RULE WEIGHT

  Give the indicated RULE the weight WEIGHT.  The WEIGHT should be
  a floating-point number.  This returns a StateLink that associates
  the weight to the rule.  The use of a StateLink means that a rule
  can have only one weight at any given time.

  The current design associates the weight with the rule name, so that
  if multiple rules have the same name, then all of thise rules will
  also have the same weight.
"
    ; FIXME - the long-term design calls for the use of an AtTimeLink
    ; instead of a StateLink, here.
    ;
    ; FIXME -- if there are multiple names for a rule, this will
    ; use all of those names in the StateLink.  If a rule is given
    ; a second or third name later on, this will cause dead StateLinks
    ; to linger in the atomspace. FIXME -- make sure that a rule can
    ; have only one name.
    ;
    (define wn (ConceptNode (string-append psi-prefix-str "weight")))
    (StateLink
        (ListLink (psi-rule-alias psi-rule) wn)
        (NumberNode weight))
)

; --------------------------------------------------------------
(define (psi-set-controlled-rule psi-rule name)
"
  psi-set-controlled-rule RULE NAME

  Specify that RULE is to be controlled and give it a NAME. Controlling means
  modifying the weight of the rule, thus affecting the likelyhood of it being choosen.
"
    (MemberLink psi-rule psi-controller-demand)

    (psi-rule-set-atomese-weight psi-rule (tv-mean (cog-tv psi-rule)))

    (psi-rule-set-alias! psi-rule name)

    psi-rule
)

; --------------------------------------------------------------
(define (psi-get-controlled-rules)
"
  Returns a list of all the rules that are going to be controlled
"
    (cog-outgoing-set
        (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "controlled-rule")
                (TypeNode "ImplicationLink"))
            (MemberLink (Variable "controlled-rule") psi-controller-demand))))
)

; --------------------------------------------------------------
(define (psi-rule-atomese-weight psi-rule)
"
  Returns a list with a NumberNode that has the weight of the given psi-rule if
  the weight is represented in atomese, if not it returns `null`.

  psi-rule:
  - An ImplicationLink whose weight is going to be modified.
"
    (cog-outgoing-set (cog-execute!
        (BindLink
            (VariableList
                (TypedVariableLink
                    (VariableNode "psi-rule-weight")
                    (TypeNode "NumberNode"))
                (TypedVariableLink
                    (VariableNode "psi-rule-alias")
                    (TypeNode "ConceptNode")))
            (AndLink
                (StateLink ; FIXME should use AtTimeLink
                    (ListLink
                        (VariableNode "psi-rule-alias")
                        (ConceptNode (string-append psi-prefix-str "weight")))
                    (VariableNode "psi-rule-weight"))
                (EvaluationLink
                    psi-rule-name-predicate-node
                    (ListLink
                        (QuoteLink psi-rule)
                        (VariableNode "psi-rule-alias"))))
            (VariableNode "psi-rule-weight"))
    ))
)

; --------------------------------------------------------------
(define (psi-controller-update-weights)
"
  This gets all the psi-rules that inherit from the psi-control-node & update
  their weight.
"
    (define (update-weight-from-atomese-weight rule)
        (let ((result (psi-rule-atomese-weight rule)))
            (if (not (null? result))
                (cog-set-tv! rule
                    (stv (string->number (cog-name (car result)))
                         (tv-conf (cog-tv rule))))
            )
        ))

    (define psi-controller-state
        (gar (cog-execute! (Get (State psi-controller (Variable "$x"))))))

    (map update-weight-from-atomese-weight
        (psi-get-rules psi-controller-demand))
)

; --------------------------------------------------------------
(define (psi-rule-disable rule-alias psi-rule-list)
"
  Disable the psi-rule which is aliased as `rule-alias`. A disabled rule is
  one with weight of zero.

  rule-alias:
  - string used for a psi-rule alias.

  psi-rule-list:
  - a list with psi-rules.
"
    (receive (filtered other)
        ; TODO: Use categories instead of aliases for categorization
        (psi-partition-rule-with-alias rule-alias psi-rule-list)
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            filtered)
    )
)

; --------------------------------------------------------------
(define (psi-rule-enable rule-alias psi-rule-list)
"
  Disable the psi-rule which is aliased as `rule-alias`. An enabled rule is
  one with weight greater than zero. Here the weight is set to 0.9

  rule-alias:
  - string used for a psi-rule alias.

  psi-rule-list:
  - a list with psi-rules.
"
    (receive (filtered other)
      ; TODO: Use categories instead of aliases for categorization
        (psi-partition-rule-with-alias rule-alias psi-rule-list)
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            filtered)
    )
)

; --------------------------------------------------------------
; Returns a SetLink with ListLink of psi-controlled-rule aliases and their
; atomese-weight.
(Define
    (DefinedSchema "psi-controlled-rule-state")
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "controlled-rule")
                (TypeNode "ImplicationLink"))
            (TypedVariableLink
                (VariableNode "psi-rule-weight")
                (TypeNode "NumberNode"))
            (TypedVariableLink
                (VariableNode "psi-rule-alias")
                (TypeNode "ConceptNode")))
        (And
            (EvaluationLink
                psi-rule-name-predicate-node
                (ListLink
                    (Variable "controlled-rule")
                    (VariableNode "psi-rule-alias")))
            (MemberLink
                (Variable "controlled-rule")
                psi-controller-demand)
            (StateLink ; FIXME should use AtTimeLink
                (ListLink
                    (VariableNode "psi-rule-alias")
                    (ConceptNode (string-append psi-prefix-str "weight")))
                (VariableNode "psi-rule-weight")))
        (List
            (VariableNode "psi-rule-alias")
            (VariableNode "psi-rule-weight"))
    )
)

; --------------------------------------------------------------
; Thought of using the updater in the psi-step but got to add additional
; functions for filtering by tags/demands
;
;(define controller-updater
;    (DefinedPredicateNode (string-append psi-prefix-str "controller")))
;
;(DefineLink
;    controller-updater
;    (EvaluationLink
;        (GroundedPredicateNode "scm: psi-controller-update-weights")
;        (ListLink))
;)
;
;(psi-set-updater! controller-updater psi-controller-demand)

; --------------------------------------------------------------
; Skip controller-demand, as it is only used to update weight of the
; rules that are associated with it.  (??? Huh ???)
(psi-demand-skip psi-controller-demand)
