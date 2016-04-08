; Copyright (C) 2015-2016 OpenCog Foundation
;
; Helper functions for OpenPsi

(use-modules (ice-9 regex)) ; For string-match

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
