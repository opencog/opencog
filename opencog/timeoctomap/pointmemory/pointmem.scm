(define-module (opencog ato pointmem))

(use-modules (srfi srfi-1) (opencog) (opencog atom-types) (opencog exec))

(load-extension "libpoint_memory" "opencog_ato_pointmem_init")
