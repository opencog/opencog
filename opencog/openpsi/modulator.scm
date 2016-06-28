; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog))

(load "utilities.scm")

; --------------------------------------------------------------
(define psi-modulator-node
    (ConceptNode (string-append psi-prefix-str "modulator")))

; --------------------------------------------------------------
(define-public (psi-modulator modulator-name)
"
  Returns the ConceptNode that represents an OpenPsi modulator

  modulator-name:
  - The name of the modulator that is created.
"
    ; Check arguments
    (if (not (string? modulator-name))
        (error (string-append "In procedure psi-modulator, expected first "
            "argument to be a string got: ") modulator-name))

    (let* ((modulator-str (string-append psi-prefix-str modulator-name))
           (modulator-node (ConceptNode modulator-str)))

       (InheritanceLink modulator-node psi-modulator-node)

        modulator-node
    )
)

; --------------------------------------------------------------
(define-public (psi-set-updater! updater tag-node)
"
  Returns the alias node that represents the updater.

  updater:
  - An evaluatable link/node which when evaluated will updater the values for
    the given demand/modulator.

  tag-node:
  - A demand/modulator node that the updater is being added to.
"
    (psi-set-functionality updater #t tag-node "updater")
)

; --------------------------------------------------------------
(define-public (psi-get-updater tag-node)
"
  Returns a list containing the updater for the given tag-node. Nil is returned
  if it doesn't have one.

  tag-node:
  - A demand/modulator node that has the updater.
"
    (psi-get-functionality tag-node "updater")
)
