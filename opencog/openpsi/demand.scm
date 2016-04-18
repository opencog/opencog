; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (rnrs sorting)) ; For sorting demands by their values.

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "utilities.scm")

; --------------------------------------------------------------
; Name of variables for common functions in this file-scope
; NOTE: Shouldn't be exported to prevent modification.
(define demand-var (VariableNode "Demand"))

; --------------------------------------------------------------
(define-public (psi-get-demands dpn)
"
  Filters demands using the DefinedPredicateNode passed as argument and
  returns a SetLink with the results.

  dpn:
  - DefinedPredicateNode that represents the evaluatable term that will filter
    demands. The evaluatable term should take a single demand-ConceptNode and
    return True-TruthValue `(stv 1 1)`  or False-TruthValue `(stv 0 1)`. The
    term should have atleast one `(VariableNode \"demand\")`.
    (Optionaly the argument could be a `(TrueLink)` for returning all the
    demands defined)
"

    (define (get-demand) (cog-bind
        (BindLink
            (AndLink
                dpn
                (InheritanceLink
                    demand-var
                    (ConceptNode (string-append (psi-prefix-str) "Demand")))
                (EvaluationLink
                    (PredicateNode
                        (string-append (psi-prefix-str) "initial_value"))
                    (ListLink
                        demand-var
                        (VariableNode "value"))))
                demand-var)))

    ; check arguments
    (if (and (not (equal? (cog-type dpn) 'DefinedPredicateNode))
             (not (equal? (cog-type dpn) 'TrueLink)))
        (error "Expected DefinedPredicateNode or TrueLink got: " dpn))

    (get-demand)
)

; --------------------------------------------------------------
(define-public (psi-get-demands-all)
"
  Returns a SetLink containing the nodes that carry the demand-value. The
  strength of their stv is the demand value.
"
    (psi-get-demands (TrueLink))
)

; --------------------------------------------------------------
(define-public (psi-demand demand-name initial-value)
"
  Define an OpenPsi demand, that will have a default behavior defined by the
  the action passed. It returns the demand-node which is a ConceptNode.

  demand-name:
  - The name of the demand that is created.

  initial-value:
  - The initial demand-value. This is the strength of the demand's
    ConceptNode stv. The confidence of the stv is always 1.

"

    ; Check arguments
    (if (not (string? demand-name))
        (error (string-append "In procedure psi-demand, expected frist argument "
            "to be a string got: ") demand-name))
    (if (or (> 0 initial-value) (< 1 initial-value))
       (error (string-append "In procedure psi-demand, expected second argument "
            "the to be within [0, 1] interval, got:") initial-value))

    (let* ((demand-str (string-append (psi-prefix-str) demand-name))
           (demand-node (ConceptNode demand-str (stv initial-value 1))))

            (InheritanceLink
                demand-node
                (ConceptNode (string-append (psi-prefix-str) "Demand")))

            ; NOTE: Not sure this is needed. Possibly use is if one wants
            ; to measure how the demand-value has changed.
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "initial_value"))
                (ListLink
                    demand-node
                    (NumberNode initial-value)))

            demand-node
    )
)
;
; --------------------------------------------------------------
(define-public (psi-demand? node)
"
  Checks whether an atom is The node that satisfies the pattern used
  to define an OpenPsi demand. Returns True-TruthValue `(stv 1 1)` if it is
  and False-TruthValue `(stv 0 1)` if it isn't.

  node:
  - The node that is being checked to see if it is a ConceptNode that represents
    a demand type.
"
    (define (demand-names)
        (map cog-name (cog-outgoing-set (psi-get-demands-all))))

    ; Check arguments
    (if (not (cog-node? node))
        (error "In procedure psi-demand?: Expected a Node got: " node))

    (if (and (member (cog-name node) (demand-names))
             (equal? (cog-type node) 'ConceptNode))
        (stv 1 1)
        (stv 0 1)
    )
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
    (if (equal? (stv 0 1) (psi-demand? atom))
        (error "Expected argument to be a demand-node, got: " atom))

    (let ((atom-strength (tv-mean (cog-tv atom)))
          (lowest-demand-value (car (list-sort < (delete-duplicates
              (map (lambda (x) (tv-mean (cog-tv x)))
                   (cog-outgoing-set (psi-get-demands-all)))))))
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
(define-public (psi-demand-goal-maximize demand-node rate)
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
        (GroundedPredicateNode "scm: psi-demand-value-maximize")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-maximize demand-node rate-node)
"
  Increases the strength of the demand by the given percentage rate.

  demand-node:
  - A node representing a demand.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value will be
    updated with, on each step.
"
    (let ((strength (tv-mean (cog-tv  demand-node)))
          (conf (tv-conf (cog-tv demand-node)))
          (rate (/ (string->number (cog-name rate-node)) 100)))
        (cog-set-tv! demand-node (stv (+ strength (* strength rate)) conf))
        (stv 1 1)
    )
)

; --------------------------------------------------------------
(define-public (psi-demand-goal-minimize demand-node rate)
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
        (GroundedPredicateNode "scm: psi-demand-value-minimize")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-minimize demand-node rate-node)
"
  Decreases the strength of the demand by the given percentage rate.

  demand-node:
  - A node representing a demand.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value will be
    updated with, on each step.
"
    (let ((strength (tv-mean (cog-tv  demand-node)))
          (conf (tv-conf (cog-tv demand-node)))
          (rate (/ (string->number (cog-name rate-node)) 100)))
        (cog-set-tv! demand-node (stv (- strength (* strength rate)) conf))
        (stv 1 1)
    )
)

; --------------------------------------------------------------
(define-public (psi-demand-goal-keep-range demand-node min-value max-value rate)
"
  Returns an ExecutionOutputLink(the action) that tries to maintain the
  demand-value within the specified range. If the demand-value is within range
  then it does nothing.

  It is mainily to be used as a default effect-type, as it can increase or
  decrease the demand value.

  demand-node:
  - A node representing a demand.

  min-value:
  - A number for the minimum acceptable demand-value.

  max-value:
  - A number for the maximum acceptable demand-value.

  rate:
  - A number for the percentage of change that a demand-value be updated with,
    on each step, should it pass the boundaries.
"
    ; Check arguments
    (if (or (> min-value max-value) (> 0 min-value) (< 1 max-value))
       (error (string-append "In procedure psi-demand-goal-keep-range expected "
            "the range to be a subset of [0, 1] interval, got: "
            "[" (number->string min-value) ", " (number->string max-value) "]"))
    )
    (if (>= 0 rate)
       (error "Expected the percentage of change for the demand value "
              "to be > 0, got: " rate))

    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value-maximize-range")
        (ListLink
            demand-node
            (NumberNode min-value)
            (NumberNode max-value)
            (NumberNode rate)))
)

(define-public (psi-demand-value-maximize-range
                    demand-node min-node max-node rate-node)
"
  Increases or decreases the strength of the demand depending on whether it is
  in between the range specified. The range is taken as an open-interval, that
  must be a subset of [0, 1] interval.

  demand-node:
  - The node that represents the demand.

  min-node:
  - A NumberNode for the lower boundary of the range.

  max-node:
  - A NumberNode for the upper boundary of the range.

  rate-node:
  - A NumberNode for the percentage of change that a demand-value be updated
    with, should it pass the boundaries. Must be greater than zero.
"
    (let ((mean (tv-mean (cog-tv  demand-node)))
          (min-value (string->number (cog-name min-node)))
          (max-value (string->number (cog-name max-node)))
          (rate (/ (string->number (cog-name rate-node)) 100)))

         ; Maximize or minmize
         (cond ((strength > max-value)
                    (psi-demand-value-minimize demand-node rate-node))
               ((strength < min-value)
                    (psi-demand-value-maximize demand-node rate-node))
         )
    )
)
