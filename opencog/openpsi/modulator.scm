; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog))

(load "utilities.scm")

; --------------------------------------------------------------
(define psi-modulator-node
    (ConceptNode (string-append psi-prefix-str "modulator")))

; --------------------------------------------------------------
(define-public (psi-create-modulator modulator-name initial-value)
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

        modulator-node
    )
)

; =============================================================================
; CREATE MODULATORS

(define arousal (psi-create-modulator "arousal" .5))
(define pos-valence (psi-create-modulator "positive valence" .5))
(define neg-valence (psi-create-modulator "negative valence" .5))
(define resolution-level (psi-create-modulator "resolution-level" .5))
(define selection-threshold (psi-create-modulator "selection-threshold" .5))
(define securing-threshold (psi-create-modulator "securing-threshold" .5))


; -------------------------------------------------------------
; Getters

(define-public (psi-get-arousal)
	(psi-get-number-value arousal))

(define-public (psi-get-pos-valence)
	(psi-get-number-value pos-valence))

(define-public (psi-get-neg-valence)
    (psi-get-number-value neg-valence))

(define-public (psi-get-resolution-level)
	(psi-get-number-value resolution-level))

(define-public (psi-get-selection-threshold)
	(psi-get-number-value selection-threshold))

(define-public (psi-get-securing-threshold)
	(psi-get-number-value securing-threshold))
