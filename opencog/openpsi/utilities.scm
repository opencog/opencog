; Copyright (C) 2015-2016 OpenCog Foundation
;
; Helper functions for OpenPsi

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For fold

; --------------------------------------------------------------
(define-public (psi-prefix-str)
"
  Returns the string used as a prefix to all OpenPsi realted atom definition
"
    "OpenPsi: "
)

; --------------------------------------------------------------
(define-public (psi-suffix-str a-string)
"
  Returns the suffix of that follows `psi-prefix-str` sub-string.

  a-string:
    - a string that should have the`psi-prefix-str`

"
    (let ((z-match (string-match (psi-prefix-str) a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" (psi-prefix-str) "\". " "Instead got:" a-string) )
        )
    )
)

; --------------------------------------------------------------
(define-public (most-weighted-atom atom-list)
"
  It returns the atom with the highest weight, weight of an atom is the product
  of the stength and confidence of the atom.

  atom-list:
  - A list of atoms to be compared.
"
    (define (weight x)
        (let ((a-stv (cog-tv x)))
            (* (tv-conf a-stv) (tv-mean a-stv))))

   (fold
       (lambda (x y) (if (> (weight y) (weight x)) y x))
       (car atom-list) atom-list
   )
)
