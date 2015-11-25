
(define-module (opencog nlp relex2logic))

(use-modules (opencog) (opencog atom-types))

; XXX FIXME The install paths need to be rationalized. This load
; path is insane.
(load "../../nlp/relex2logic/utilities.scm")

(load-r2l-rulebase)
