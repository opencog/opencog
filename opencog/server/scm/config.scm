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
(use-modules (ice-9 readline))
(activate-readline)

; --------------------------------------------------------------
; Default OpenCog modules loaded
; --------------------------------------------------------------
; For cog-execute!
(use-modules (opencog exec))
