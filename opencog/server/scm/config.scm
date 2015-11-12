 ; This file is used to configure the guile environment similar to .guile
 ; file as the cogserver doesn't guile configuration shouldn't depend on the
 ; host configuration as there might be multiple instances of a cogserver
 ; running.
 ; Reference: https://github.com/opencog/atomspace/issues/87#issuecomment-111267545

; --------------------------------------------------------------
; Guile related configurations
; --------------------------------------------------------------
; Add the path where opencog's non-extenstion modules are installed.
(add-to-load-path "/usr/local/share/opencog/scm")

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
(use-modules (opencog))

; Load cog-execute! and other execution/evaluation related functions.
(use-modules (opencog exec))
