;
; Opencog atom-types module
;
(define-module (opencog atom-types))

; Alternately, we could also have
; (define-module (opencog atomtypes nlp-types))
; (define-module (opencog atomtypes spacetime-types))
; and so on, but I don't see the point of that, at the moment...

; XXX Python bug work-around -- python is unable to locate shared
; libraries.  Work around this here, by explicitly setting the
; library path, and the scheme file search path. This is NOT needed
; by the scheme code; but python croaks without this. Go figure...
(setenv "LD_LIBRARY_PATH" "/usr/local/lib/opencog:/usr/local/lib/opencog/modules")
(add-to-load-path "/usr/local/share/opencog/scm")

; Load the C libraries that actually call the classserver to load
; the types.
(load-extension "libnlp-types" "nlp_types_init")
(load-extension "libspacetime-types" "spacetime_types_init")
(load-extension "libattention-types" "attention_types_init")
(load-extension "libembodiment-types" "embodiment_types_init")

(load-from-path "nlp/types/nlp_types.scm")
(load-from-path "spacetime/spacetime_types.scm")
(load-from-path "dynamics/attention/attention_types.scm")
(load-from-path "embodiment/embodiment_types.scm")
