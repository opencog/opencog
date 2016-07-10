(setenv "LTDL_LIBRARY_PATH"
    (if (getenv "LTDL_LIBRARY_PATH")
        (string-append (getenv "LTDL_LIBRARY_PATH")
            ":/usr/local/lib/opencog:/usr/local/lib/opencog/modules")
        "/usr/local/lib/opencog:/usr/local/lib/opencog/modules"))

(define-module (opencog ato pointmem))

(use-modules (srfi srfi-1) (opencog) (opencog atom-types) (opencog exec))

(load-extension "libpoint_memory" "opencog_ato_pointmem_init")

;utility functions

;;Convert from at location link to (x y z) list
(define-public (space-nodes at-loc-link) (cog-outgoing-set (cadr (cog-outgoing-set (cadr (cog-outgoing-set at-loc-link))))))
(define-public (loc-link-x at-loc-link) 
		(string->number (cog-name (car (space-nodes at-loc-link))))
)
(define-public (loc-link-y at-loc-link)
                (string->number (cog-name (cadr (space-nodes at-loc-link))))
)
(define-public (loc-link-z at-loc-link)
                (string->number (cog-name (caddr (space-nodes at-loc-link))))
)
