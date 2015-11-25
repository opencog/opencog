
(define-module (opencog nlp relex2logic))

(use-modules (opencog) (opencog atom-types))

; XXX FIXME The install paths need to be rationalized. This load
; path is insane.
(load "relex2logic/utilities.scm")


; -----------------------------------------------------------------------
; This loads all the rules into the cogserver shell.
(define (load-r2l-rulebase)

	; "." in case the cogserver is started from in-source build directory.
	(add-to-load-path ".")
	(load "relex2logic/loader/load-rules.scm")
	(load "relex2logic/loader/gen-r2l-en-rulebase.scm")
)
(load-r2l-rulebase)
