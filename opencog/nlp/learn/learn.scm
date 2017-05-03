;
; Language learning module.
; Wraps up the assorted tools and scripts into one module.
;
(define-module (opencog nlp learn))

(load "learn/common.scm")
(load "learn/compute-mi.scm")
(load "learn/base-stats.scm") ; load after compute-mi.scm
(load "learn/disjunct-mi.scm")
(load "learn/link-pipeline.scm")
(load "learn/make-disjuncts.scm")
(load "learn/mst-parser.scm")
(load "learn/word-pair-mi.scm")
