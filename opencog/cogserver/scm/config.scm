; This file is used to configure the guile environment similar to .guile
; file as the cogserver's configuration shouldn't depend on the
; host configuration as there might be multiple instances of a cogserver
; running.
;
; NOTE: This file is only for use during developement.
;
; Reference:
; https://github.com/opencog/atomspace/issues/87#issuecomment-111267545
;
; Usage:
; * Start cogserver from the build directory using 'lib/developement.conf' for
;   for configuration file.
;
; --------------------------------------------------------------
; Guile related configurations
; --------------------------------------------------------------
(add-to-load-path (string-append (getcwd)))
(add-to-load-path (string-append (getcwd) "/opencog/scm"))
(define (add-to-ltdl-path path)
"
  Adds the the given path to `LTDL_LIBRARY_PATH` environment variable. For
  c++ extensions of guile.

  path:
  - absolute directory path taking the build directory as root
"
    (setenv "LTDL_LIBRARY_PATH"
        (if (getenv "LTDL_LIBRARY_PATH")
            (string-append (getenv "LTDL_LIBRARY_PATH") ":" path)
            path
        )
    )
)

; New atom types
(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/types"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/spacetime"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/attention"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/embodiment"))

; Scheme wrappers for c++ code
(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/fuzzy"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/sureal"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/lg-dict"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/planning"))

; --------------------------------------------------------------
; Default guile modules loaded
; --------------------------------------------------------------
; To make working with arrow keys easier
; TODO: Figure out why this doesn't work.
; For now continue using `rlwrap telnet ...`
;(use-modules (ice-9 readline))
;(activate-readline)

; --------------------------------------------------------------
; Default OpenCog modules loaded
; --------------------------------------------------------------

; Load core-types and other utility functions.
; NOTE: Must be loaded first to avoid error likes
; ERROR: In procedure dynamic-link: file: "libruleengine", message: "file not found"
; TODO: It seems weird that one-module has to be imported to use another one.
;(use-modules (opencog))

; Load cog-execute! and other execution/evaluation related functions.
;(use-modules (opencog exec))
