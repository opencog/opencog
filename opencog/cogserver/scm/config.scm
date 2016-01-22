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
; 1. Start cogserver using 'lib/developement.conf' file
; 2. In the scheme shell run
;    `(add-to-load-path (string-append (getcwd) "/opencog/scm"))`

; --------------------------------------------------------------
; Guile related configurations
; --------------------------------------------------------------
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

(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/types"))
(add-to-ltdl-path (string-append (getcwd) "/opencog/nlp/fuzzy"))
(add-to-load-path (string-append (getcwd) "/opencog/nlp/sureal"))
(add-to-load-path (string-append (getcwd) "/opencog/nlp/lg-dict"))

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
; TODO: some of the modules loaded by SCM_PRELOAD in opencog.conf can be
; removed and imported using (use-modules), so
; 1. Figure out which one this are and remove them and replace them with an
;    import in this file
; 2. Make sure this works when cogserver is installed
; 3. Document the steps needed when installing atomspace/cogutils/opencog
;    in non-standard locations.

; Load core-types and other utility functions.
; NOTE: Must be loaded first to avoid error likes
; ERROR: In procedure dynamic-link: file: "libruleengine", message: "file not found"
; TODO: It seems weird that one-module has to be imported to use another one.
;(use-modules (opencog))

; Load cog-execute! and other execution/evaluation related functions.
;(use-modules (opencog exec))
