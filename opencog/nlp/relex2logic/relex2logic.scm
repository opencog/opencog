
(define-module (opencog nlp relex2logic))

; (add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog atom-types) (opencog nlp))

(load "relex2logic/utilities.scm")
(load "relex2logic/loader/load-rules.scm")
(load "relex2logic/loader/gen-r2l-en-rulebase.scm")

; -----------------------------------------------------------------------
; This loads all the rules into the cogserver shell.
(define (load-r2l-rulebase)

	; "." in case the cogserver is started from in-source build directory.
	(add-to-load-path ".")
	(load "relex2logic/loader/load-rules.scm")
	(load "relex2logic/loader/gen-r2l-en-rulebase.scm")
)
