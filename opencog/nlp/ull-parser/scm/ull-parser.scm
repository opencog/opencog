;
; Language learning module.
; Wraps up the assorted tools and scripts into one module.
;
(define-module (opencog nlp ull-parser))

; The files are loaded in pipeline order.
; In general, the later files depend on definitions contained
; in the earlier files.
(load "ull-parser/batch-word-pair.scm")
(load "ull-parser/common.scm")
(load "ull-parser/link-pipeline.scm")
(load "ull-parser/mst-parser.scm")
(load "ull-parser/singletons.scm")
(load "ull-parser/summary.scm")
