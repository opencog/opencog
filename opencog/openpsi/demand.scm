; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (rnrs sorting)) ; For sorting demands by their values.

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "utilities.scm")

; --------------------------------------------------------------
; Name of variables for common functions in this file-scope
; NOTE: Shouldn't be exported to prevent modification.
(define demand-var (VariableNode "Demand"))

(define psi-demand-node (ConceptNode (string-append psi-prefix-str "Demand")))

; --------------------------------------------------------------
; A cache of the demands for performance.
(define psi-valid-demand-cache '())

; --------------------------------------------------------------
(define-public (psi-reset-valid-demand-cache)
"
  Reset the cache for psi-demand-cache.
"
    (set! psi-valid-demand-cache '())
)
; --------------------------------------------------------------
; --------------------------------------------------------------
(define-public (psi-get-all-demands)
"
  Returns a list containing all the demand-nodes.
"
    (filter
        (lambda (x) (not (equal? x psi-demand-node)))
        (cog-chase-link 'InheritanceLink 'ConceptNode psi-demand-node))
)


; --------------------------------------------------------------
(define-public (psi-get-all-valid-demands)
"
  Returns a list containing all the demand-nodes that are valid. A valid demand
  is one which is not a member of the set defined by
  (ConceptNode \"OpenPsi: skip\").
"
    (if (null? psi-valid-demand-cache)
        (begin
            (set! psi-valid-demand-cache
                (filter (lambda (x) (not (psi-demand-skip? x)))
                    (psi-get-all-demands))
            )
            psi-valid-demand-cache
        )
        psi-valid-demand-cache
    )
)

; --------------------------------------------------------------
(define-public (psi-demand demand-name desired-value)
"
  Returns the ConceptNode that represents an OpenPsi demand

  demand-name:
  - The name of the demand that is created.

  desired-value:
  - The desired demand-value. This is used for setting goals.
"

    ; Check arguments
    (if (not (string? demand-name))
        (error (string-append "In procedure psi-demand, expected first argument "
            "to be a string got: ") demand-name))
    (if (or (> 0 desired-value) (< 1 desired-value))
       (error (string-append "In procedure psi-demand, expected second argument "
            "the to be within [0, 1] interval, got:") desired-value))

    (let* ((demand-str (string-append psi-prefix-str demand-name))
           (demand-node (ConceptNode demand-str (stv desired-value 1))))

            (InheritanceLink demand-node psi-demand-node)

            ; NOTE: Not sure this is needed. Possibly use is if one wants
            ; to measure how the demand-value has changed.
            (EvaluationLink
                (PredicateNode (string-append psi-prefix-str "desired_value"))
                (ListLink
                    demand-node
                    (NumberNode desired-value)))

            demand-node
    )
)

; --------------------------------------------------------------
(define-public (psi-demand? node)
"
  Checks whether an atom is a node that satisfies the pattern used
  to define an OpenPsi-demand and returns `#t` if it does and `#f` if not.

  node:
  - The node that is being checked to see if it is a ConceptNode that represents
    a demand type.
"
    (let ((candidates (cog-chase-link 'InheritanceLink 'ConceptNode node)))

        ; A filter is used to account for empty list as well as
        ; cog-chase-link returning multiple results, just in case.
        (not (null?
            (filter (lambda (x) (equal? x psi-demand-node)) candidates)))
    )
)

; --------------------------------------------------------------
(define-public (psi-set-demand-value demand-node demand-value)
"
  Sets the demand-value of demand-node.

  demand-node:
  - The node representing the demand.

  demand-value:
  - A number between [0, 1] that is used to set the demand-value.
"
; NOTE: This function is added to simplify the process of setting demand-value
; when the move is made to represent that in atomese rather than stv.
    (cog-set-tv! demand-node (stv demand-value (tv-conf (cog-tv demand-node))))
)

; --------------------------------------------------------------
(define (psi-demand-value-term< threshold)
"
  Returns an evaluatable term that checks if a demand has value less than
  the given threshold number.

  threshold:
  - The boundary of the demand-value to be checked at.
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value<")
        (ListLink
            demand-var
            (NumberNode threshold)))
)

(define (psi-demand-value< demand-node threshold-node)
"
  Returns True-TruthValue if a given demand-node has value less than the
  given threshold-node number value else False-TruthValue. This doesn't
  check if the node given actually defines a demand. And is primarily to be
  used as evaluatable term.

  demand-node:
  - The node representing the demand.

  threshold-node:
  - A NumberNode representing the boundary of the demand-value to be checked
    at.
"
    (if (< (tv-mean (cog-tv demand-node))
           (string->number (cog-name threshold-node)))
        (stv 1 1)
        (stv 0 1)
    )
)

; --------------------------------------------------------------
(define (psi-demand-value-term> threshold)
"
  Returns an evaluatable term that checks if a demand has value greater than
  the given threshold number.

  threshold:
  - The boundary of the demand-value to be checked at.
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value>")
        (ListLink
            demand-var
            (NumberNode threshold)))
)

(define (psi-demand-value> demand-node threshold-node)
"
  Returns True-TruthValue if a given demand-node has value greater than the
  given threshold-node number value else False-TruthValue. This doesn't
  check if the node given actually defines a demand. And is primarily to be
  used as evaluatable term.

  demand-node:
  - The node representing the demand.

  threshold-node:
  - A NumberNode representing the boundary of the demand-value to be checked
    at.
"
    (if (> (tv-mean (cog-tv demand-node))
           (string->number (cog-name threshold-node)))
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
    (if (not (psi-demand? atom))
        (error "Expected argument to be a demand-node, got: " atom))

    (let ((atom-strength (tv-mean (cog-tv atom)))
          (lowest-demand-value (car (list-sort < (delete-duplicates
              (map (lambda (x) (tv-mean (cog-tv x)))
                   (psi-get-all-demands))))))
         )
         (if (<= atom-strength lowest-demand-value)
            (stv 1 1)
            (stv 0 1)
         )
    )
)

; --------------------------------------------------------------
; Functions to help define standard action-rules
; --------------------------------------------------------------
(define-public (psi-goal-increase demand-node rate)
"
  Returns an ExecutionOutputLink(the action) that increases the demand-value.
  It has an increasing effect on the demand-value.

  demand-node:
  - A node representing a demand.

  rate:
  - A number for the percentage of change that a demand-value will be updated
    with, on each step.
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value-increase")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-increase demand-node rate-node)
"
  Increases the strength of the demand by the given percentage rate.

  demand-node:
  - A node representing a demand.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value will be
    updated with, on each step.
"
    (let* ((strength (tv-mean (cog-tv  demand-node)))
           (rate (/ (string->number (cog-name rate-node)) 100))
           (demand-value (+ strength (* strength rate))))
        (psi-set-demand-value demand-node demand-value)
        (stv 1 1)
    )
)

; --------------------------------------------------------------
(define-public (psi-goal-decrease demand-node rate)
"
  Returns an ExecutionOutputLink(the action) that decreases the demand-value.
  It has a decreasing effect on the demand value.

  demand-node:
  - A node representing a demand.

  rate:
  - A number for the percentage of change that a demand-value will be updated
    with, on each step.
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value-decrease")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-decrease demand-node rate-node)
"
  Decreases the strength of the demand by the given percentage rate.

  demand-node:
  - A node representing a demand.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value will be
    updated with, on each step.
"
    (let* ((strength (tv-mean (cog-tv  demand-node)))
           (rate (/ (string->number (cog-name rate-node)) 100))
           (demand-value (- strength (* strength rate))))
        (psi-set-demand-value demand-node demand-value)
        (stv 1 1)
    )
)

; --------------------------------------------------------------
; This is used to label a demand for skipping. During actions-selection.
(define psi-label-skip (ConceptNode (string-append psi-prefix-str "skip")))

; --------------------------------------------------------------
(define-public (psi-demand-skip demand)
"
  This is used to label a demand as one to be skipped during action-selection.

  demand:
  - A ConceptNode, representing a demand that should be skipped during
   action-selection.
"
    ; Check arguments
    (if (not (psi-demand? demand))
        (error (string-append "In procedure psi-demand-skip, expected "
            "argument to be a node representing a demand, got:") demand))

    (MemberLink demand psi-label-skip)
)

; --------------------------------------------------------------
(define-public (psi-demand-skip? demand)
"
  Check if the passed atom is skippable demand and return `#t`, if it is,
  and `#f` otherwise. An atom is an skippable if it a member of the set
  represented by (ConceptNode \"OpenPsi: skip\").
"
    (define (get-skipped-demands)
        (cog-outgoing-set (cog-execute!
            (GetLink
                (TypedVariableLink
                    (VariableNode "demand")
                    (TypeNode "ConceptNode"))
                (AndLink
                    (InheritanceLink (VariableNode "demand") psi-demand-node)
                    (MemberLink (VariableNode "demand") psi-label-skip)))
        )))

    ; Check arguments
    (if (not (psi-demand? demand))
        (error (string-append "In procedure psi-demand-skip?, expected "
            "argument to be a node representing a demand, got:") demand))

    (let ((candidates (get-skipped-demands)))
        (if (member demand candidates) #t #f)
    )
)
