;
; Opencog atom-types module
;
(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH")
            ":/usr/local/lib/opencog")
        "/usr/local/lib/opencog"))

(define-module (opencog atom-types))
; Alternately, we could also have
; (define-module (opencog atomtypes spacetime-types))
; and so on, but I don't see the point of that, at the moment...

; Load the C libraries that call the classserver to load the types.
(load-extension "libattention-types" "attention_types_init")
(load-extension "libspacetime-types" "spacetime_types_init")

(use-modules (opencog))  ; needed for cog-type->int
(load "attention/attention_types.scm")
(load "spacetime/spacetime_types.scm")
