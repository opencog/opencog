;
; demand.scm
; Methods to define and work with demands.
;
; Copyright (C) 2015-2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud
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
(define psi-demand-node (ConceptNode (string-append psi-prefix-str "demand")))

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

    ; Compute the list of enabled demands, and cache it.
    (set! psi-demand-cache
        (lset-difference! equal? (psi-get-all-demands)
            (cog-outgoing-set skip-set))
    )

    ; Delete the SetLink, so it does not pollute the atomspace.
    (cog-delete skip-set)
)

; --------------------------------------------------------------
(define (psi-get-all-demands)
"
  psi-get-all-demands - Return a list of all demand-nodes.
"
    (filter
        (lambda (x) (not (equal? x psi-demand-node)))
        (cog-chase-link 'InheritanceLink 'ConceptNode psi-demand-node))
)


; --------------------------------------------------------------
(define (psi-get-all-enabled-demands)
"
  psi-get-all-enabled-demands - Return list of all demands that are enabled.
"
    (if (null? psi-demand-cache) (make-demand-cache))
    psi-demand-cache
)

; Backwards-compat wrapper
(define (psi-get-all-valid-demands)
"  Do not use this, use psi-get-all-enabled-demands instead "
(psi-get-all-enabled-demands))

; --------------------------------------------------------------
(define (psi-demand NAME)
"
  psi-demand NAME VALUE

  Create and return a ConceptNode that represents an OpenPsi demand.
  The NAME should be a string.
"
    (let* ((demand-node (ConceptNode NAME)))
        (InheritanceLink demand-node psi-demand-node)
        demand-node
    )
)

; --------------------------------------------------------------
(define (psi-demand? node)
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
(define (psi-set-demand-value demand-node demand-value)
"
  psi-set-demand-value DEMAND VALUE - Set the DEMAND to VALUE.

  The DEMAND must be a demand node that was previously declared to
  the system. The VALUE must be a floating-point number between [0, 1].
"
    (cog-set-tv! demand-node (stv demand-value (tv-conf (cog-tv demand-node))))
)

; --------------------------------------------------------------
; Functions to help define standard action-rules
; --------------------------------------------------------------
(define (psi-demand-value-increase demand-node rate-node)
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
(define (psi-demand-value-decrease demand-node rate-node)
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
(define (psi-demand-skip demand)
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
(define (psi-demand-skip? demand)
"
  psi-demand-skip? DEMAND - return #t if DEMAND should be skipped.
"
    ; Check arguments
    (if (not (psi-demand? demand))
        (error (string-append "In procedure psi-demand-skip?, expected "
            "argument to be a node representing a demand, got:") demand))

    (not (member demand (psi-get-all-enabled-demands)))
)
