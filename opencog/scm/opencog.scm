;
; Main OpenCog guile module
;

(define-module (opencog))
(load-extension "libsmob" "opencog_guile_init")

(use-modules (system base compile))

; Initialze a default atomspace, just to keep things sane...
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

