;
; Correlation matrix analsysis module.
; Wraps up the assorted tools and scripts into one module.
;
(define-module (opencog analysis))

; The files are loaded in pipeline order.
; In general, the later files depend on definitions contained
; in the earlier files.
(load "analysis/object-api.scm")
(load "analysis/fold-api.scm")
(load "analysis/support.scm")
(load "analysis/entropy.scm")
(load "analysis/compute-mi.scm")

