;
; modulator.scm
;
; Modulators - where is this documented?
;
; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog exec))

(load "utilities.scm")

; --------------------------------------------------------------
(define psi-modulator-node
	(ConceptNode (string-append psi-prefix-str "modulator")))

; --------------------------------------------------------------
(define (psi-create-modulator modulator-name initial-value)
"
  Creates and returns ConceptNode that represents an OpenPsi modulator

  modulator-name:
  - The name of the modulator that is created.
  initial-value:
  - In [0,1]
"
	; Check arguments
	(if (not (string? modulator-name))
		(error (string-append "In procedure psi-modulator, expected first "
			"argument to be a string got: ") modulator-name))
	(if (or (< initial-value 0) (> initial-value 1))
		(error (string-append "In procedure psi-modulator, expected second "
			"argument to be in [0,1], got: ") modulator-name))

	(let* ((modulator-str (string-append psi-prefix-str modulator-name))
		   (modulator-node (ConceptNode modulator-str)))

		(InheritanceLink modulator-node psi-modulator-node)
		(psi-set-value! modulator-node initial-value)

		; Set the (preferred) baseline vaue as the initial value
		(psi-set-baseline-value! modulator-node initial-value)

		modulator-node
	)
)

(define (psi-get-modulators)
	; todo: cog-chase-link bug? - it is returning the anchor node in the results
	;(cog-chase-link 'InheritanceLink 'ConceptNode psi-modulator-node))
	(cog-outgoing-set (cog-execute!
		(Get (Inheritance (Variable "$mod") psi-modulator-node)))))

; =============================================================================
; CREATE MODULATORS

(define default-modulators '())

(define (add-to-default-modulators NAME MODULATOR)
	(set! default-modulators (assoc-set! default-modulators NAME MODULATOR))
)

(add-to-default-modulators "arousal" (psi-create-modulator "arousal" .5))
(add-to-default-modulators "pos-valence"
	(psi-create-modulator "positive-valence" .5))
(add-to-default-modulators "neg-valence"
	(psi-create-modulator "negative-valence" .5))
(add-to-default-modulators "resolution-level"
	(psi-create-modulator "resolution-level" .5))
(add-to-default-modulators "selection-threshold"
	(psi-create-modulator "selection-threshold" .5))
(add-to-default-modulators "securing-threshold"
	(psi-create-modulator "securing-threshold" .5))
(add-to-default-modulators "goal-directedness"
	(psi-create-modulator "goal-directedness" .5))

(define (psi-default-modulator-alist)
"
	psi-default-modulator-alist

	Returns an alist with modulator-names for keys and modulator-nodes for values.
"
	default-modulators
)

; -------------------------------------------------------------
; Getters

(define (psi-get-arousal)
	(psi-get-number-value (assoc-ref default-modulators "arousal")))

(define (psi-get-pos-valence)
	(psi-get-number-value (assoc-ref default-modulators "pos-valence")))

(define (psi-get-neg-valence)
	(psi-get-number-value (assoc-ref default-modulators "neg-valence")))

(define (psi-get-resolution-level)
	(psi-get-number-value (assoc-ref default-modulators "resolution-level")))

(define (psi-get-selection-threshold)
	(psi-get-number-value (assoc-ref default-modulators "selection-threshold")))

(define (psi-get-securing-threshold)
	(psi-get-number-value (assoc-ref default-modulators "securing-threshold")))
