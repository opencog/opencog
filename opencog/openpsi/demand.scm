;
; demand.scm
; Methods to define and work with demands.
;
; Copyright (C) 2015-2016 OpenCog Foundation
;
; Design Notes:
; Demands are associated with demand values. There are two different,
; conflicting ways in which this is done in the code below: in some
; cases, the value is stored as the mean of the TV; in other cases, the
; value is a NumberNode attached to the ConceptNode. Having two places
; to store this info is just asking for hard-to-find bugs to pop up.
; The long-term solution is probably to associate the value to the
; demand using a protoatom.

(use-modules (rnrs sorting)) ; For sorting demands by their values.
(use-modules (srfi srfi-1)) ; for `lset-difference`

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "utilities.scm")

; --------------------------------------------------------------
(define psi-demand-node (ConceptNode (string-append psi-prefix-str "Demand")))

; --------------------------------------------------------------
; A cache of the demands, used to improve performance.
(define psi-demand-cache '())

; --------------------------------------------------------------
; reset the cache
(define (clear-demand-cache)
    (set! psi-demand-cache '())
)

(define (make-demand-cache)

    ; Get the set of demands that are currently disabled.
    (define skip-set
        (cog-execute!
            (GetLink
                (TypedVariableLink
                    (VariableNode "demand")
                    (TypeNode "ConceptNode"))
                (AndLink
                    (InheritanceLink (VariableNode "demand") psi-demand-node)
                    (MemberLink (VariableNode "demand") psi-label-skip)))
        ))

    ; Compute the list of enabled deamnds, and cache it.
    (set! psi-demand-cache
        (lset-difference! equal? (psi-get-all-demands)
            (cog-outgoing-set skip-set))
    )

    ; Delete the SetLink, so it does not pollute the atomspace.
    (cog-delete skip-set)
)

; --------------------------------------------------------------
(define-public (psi-get-all-demands)
"
  psi-get-all-demands - Return a list of all demand-nodes.
"
    (filter
        (lambda (x) (not (equal? x psi-demand-node)))
        (cog-chase-link 'InheritanceLink 'ConceptNode psi-demand-node))
)


; --------------------------------------------------------------
(define-public (psi-get-all-enabled-demands)
"
  psi-get-all-enabled-demands - Return list of all demands that are enabled.
"
    (if (null? psi-demand-cache) (make-demand-cache))
    psi-demand-cache
)

; Backwards-compat wrapper
(define-public (psi-get-all-valid-demands)
"  Do not use this, use psi-get-all-enabled-demands instead "
(psi-get-all-enabled-demands))

; --------------------------------------------------------------
(define-public (psi-demand demand-name desired-value)
"
  psi-demand NAME VALUE

  Create and return a ConceptNode that represents an OpenPsi demand.
  The NAME should be a string. The VALUE should be a floating-point
  number in the range [0,1].
"

    ; Check arguments
    (if (not (string? demand-name))
        (error "ERROR: psi-demand, expected first argument to be a string"))

    (if (or (> 0 desired-value) (< 1 desired-value))
       (error (string-append
            "ERROR: psi-demand, expected second argument to be a number "
            "in the range [0,1], got:") desired-value))

    (let* ((demand-str (string-append psi-prefix-str demand-name))
           (demand-node (ConceptNode demand-str (stv desired-value 1))))

        (InheritanceLink demand-node psi-demand-node)

        ; NOTE: Not sure that this link is needed. One possible use is
        ; to measure how the demand-value has changed over time.  XXX
        ; Is this actaully used anywhere?
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
  psi-demand? NODE - Return #t if NODE is a demand, else return #f.

  The NODE is a demand only if it was previously added to the set of
  demands.
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
  psi-set-demand-value DEMAND VALUE - Set the DEMAND to VALUE.

  The DEMAND must be a demand node that was previously declared to
  the system. The VALUE must be a floating-point number between [0, 1].
"
    (cog-set-tv! demand-node (stv demand-value (tv-conf (cog-tv demand-node))))
)

; ; --------------------------------------------------------------
; Not used anywhere.
;
; (define (psi-demand-value-term< threshold)
; "
;   Returns an evaluatable term that checks if a demand has value less than
;   the given threshold number.
;
;   threshold:
;   - The boundary of the demand-value to be checked at.
; "
;     (EvaluationLink
;         (GroundedPredicateNode "scm: psi-demand-value<")
;         (ListLink
;             (VariableNode "Demand")
;             (NumberNode threshold)))
; )
;
; (define (psi-demand-value< demand-node threshold-node)
; "
;   Returns True-TruthValue if a given demand-node has value less than the
;   given threshold-node number value else False-TruthValue. This doesn't
;   check if the node given actually defines a demand. And is primarily to be
;   used as evaluatable term.
;
;   demand-node:
;   - The node representing the demand.
;
;   threshold-node:
;   - A NumberNode representing the boundary of the demand-value to be checked
;     at.
; "
;     (if (< (tv-mean (cog-tv demand-node))
;            (string->number (cog-name threshold-node)))
;         (stv 1 1)
;         (stv 0 1)
;     )
; )
;
; --------------------------------------------------------------
; Not used anywhere.
;
; (define (psi-demand-value-term> threshold)
; "
;   Returns an evaluatable term that checks if a demand has value greater than
;   the given threshold number.
;
;   threshold:
;   - The boundary of the demand-value to be checked at.
; "
;     (EvaluationLink
;         (GroundedPredicateNode "scm: psi-demand-value>")
;         (ListLink
;             (VariableNode "Demand")
;             (NumberNode threshold)))
; )
;
; (define (psi-demand-value> demand-node threshold-node)
; "
;   Returns True-TruthValue if a given demand-node has value greater than the
;   given threshold-node number value else False-TruthValue. This doesn't
;   check if the node given actually defines a demand. And is primarily to be
;   used as evaluatable term.
;
;   demand-node:
;   - The node representing the demand.
;
;   threshold-node:
;   - A NumberNode representing the boundary of the demand-value to be checked
;     at.
; "
;     (if (> (tv-mean (cog-tv demand-node))
;            (string->number (cog-name threshold-node)))
;         (stv 1 1)
;         (stv 0 1)
;     )
; )
;
;; --------------------------------------------------------------
;;
;; Not used anywhere. Not clear how this could even be useful.
;;
;; (define (psi-lowest-demand? atom)
;; "
;;   psi-lowest-demand? ATOM - Return #t if ATOM is a demand, and has a
;;   demand-value as low or lower than any other demand.
;; "
;;     ; check if atom is a demand-node
;;     (if (not (psi-demand? atom))
;;         (error "Expected argument to be a demand-node, got: " atom))
;;
;;     (let ((atom-strength (tv-mean (cog-tv atom)))
;;           (lowest-demand-value (car (list-sort < (delete-duplicates
;;               (map (lambda (x) (tv-mean (cog-tv x)))
;;                    (psi-get-all-demands))))))
;;          )
;;          (if (<= atom-strength lowest-demand-value)
;;             (stv 1 1)
;;             (stv 0 1)
;;          )
;;     )
;; )
;;
; --------------------------------------------------------------
; Functions to help define standard action-rules
; --------------------------------------------------------------
(define-public (psi-goal-increase demand-node rate)
"
  psi-goal-increase DEMAND RATE

  Return an action that increases the satsifaction of a demand.
  That is, if the action is performed, then the value of the demand
  goes up.  XXX WTF?? this seems backward.

  rate:
  - A number for the percentage of change that a demand-value will be updated
    with, on each step. XXX WTF ???
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value-increase")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-increase demand-node rate-node)
"
  psi-demand-value-increase DEMAND RATE

  Increases the strength of DEMAND by the given percentage RATE.

  RATE must be a NumberNode, holding the percentage change by which
  the demand-value will be updated with, on each step.
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
  psi-goal-decrease DEMAND RATE

  Returns an action that, if performed, will decrease the value of
  DEMAND.

  RATE must be a floating-point number, holding a percentage value
  by which the demand will be changed on each step.
"
    (EvaluationLink
        (GroundedPredicateNode "scm: psi-demand-value-decrease")
        (ListLink
            demand-node
            (NumberNode rate)))
)

(define-public (psi-demand-value-decrease demand-node rate-node)
"
  psi-demand-value-decrease DEMAND RATE

  Decreases the strength of the demand by the given percentage rate.

  RATE must be a NumberNode holding the percentage change by which
  the demand-value will be updated with, on each step.
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
  psi-demand-skip DEMAND - Disable DEMAND.

  Use this to disable the DEMAND from being considered during action
  selection.  The DEMAND should be a valid demand node.
"
    ; Check arguments
    (if (not (psi-demand? demand))
        (error (string-append "In procedure psi-demand-skip, expected "
            "argument to be a node representing a demand, got:") demand))

    (MemberLink demand psi-label-skip)
    (clear-demand-cache)
)

; --------------------------------------------------------------
(define-public (psi-demand-skip? demand)
"
  psi-demand-skip? DEMAND - return #t if DEMAND should be skipped.
"
    ; Check arguments
    (if (not (psi-demand? demand))
        (error (string-append "In procedure psi-demand-skip?, expected "
            "argument to be a node representing a demand, got:") demand))

    (not (member demand (psi-get-all-enabled-demands)))
)
