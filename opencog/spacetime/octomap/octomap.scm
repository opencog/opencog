(define-module (opencog octomap))

(use-modules (srfi srfi-1) (opencog) (opencog exec))
(use-modules (opencog oc-config))

; Load the C library that calls the nameserver to load the types.
(load-extension (string-append opencog-ext-path-point-memory "liboctomap-types") "octomap_types_init")
(load "spacetime/octomap/octomap_types.scm")
