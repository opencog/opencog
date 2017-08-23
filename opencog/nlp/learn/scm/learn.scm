;
; Language learning module.
; Wraps up the assorted tools and scripts into one module.
;
(define-module (opencog nlp learn))

; The files are loaded in pipeline order.
; In general, the later files depend on definitions contained
; in the earlier files.
(load "learn/common.scm")
(load "learn/link-pipeline.scm")
(load "learn/singletons.scm")
(load "learn/batch-word-pair.scm")
(load "learn/mst-parser.scm")
(load "learn/pseudo-csets.scm")
(load "learn/summary.scm")
