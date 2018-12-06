(define-module (opencog octomap))

(use-modules (srfi srfi-1) (opencog) (opencog exec))

; Load the C library that calls the nameserver to load the types.
(load-extension "libatom_octomap" "opencog_spacetime_octomap_init")
(load-extension "liboctomap-types" "octomap_types_init")
(load "spacetime/octomap/octomap_types.scm")
