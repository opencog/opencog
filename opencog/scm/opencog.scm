;
; Main OpenCog guile module
;
; When this module is loaded from the guile prompt, it sets up all of
; the opencog infrastructure, including a default atomspace.
;
; To use, say this from the guile prompt:
; (load-modules (opencog))
;

(define-module (opencog))
(load-extension "libsmob" "opencog_guile_init")

(use-modules (system base compile))

; Initialze a default atomspace, just to keep things sane...
; The below is safe, because this module runs at most only once
; (if invoked from the guile shell, as (load-modules (opencog)) )
; or zero times, if invoked from the cogserver shell. The cogserver
; refuses to run this; the cogserver main atomspace is never clobbered.
;
(cog-set-atomspace! (cog-new-atomspace))

; Load a bunch of atom types too ...
; I suppose we could put all this into (define-module (opencog atomtypes))
; but I don't see a strong reason for doing this, at the moment.
; Alternately, we could also have
; (define-module (opencog atomtypes core-types))
; (define-module (opencog atomtypes nlp-types))
; and so on, but I don't see the point of that either, at the moment...
(load-from-path "./opencog/atomspace/core_types.scm")
(load-from-path "./opencog/nlp/types/nlp_types.scm")
(load-from-path "./opencog/spacetime/spacetime_types.scm")
(load-from-path "./opencog/reasoning/pln/pln_types.scm")
(load-from-path "./opencog/dynamics/attention/attention_types.scm")
(load-from-path "./opencog/embodiment/AtomSpaceExtensions/embodiment_types.scm")

; Load other grunge too
; Lots of these things should probably be modules ...
; Also they need to be defined "public" to be useable in guile...
(load-from-path "config.scm")
(load-from-path "utilities.scm")
(load-from-path "apply.scm")
(load-from-path "av-tv.scm")
(load-from-path "file-utils.scm")
(load-from-path "persistence.scm")

