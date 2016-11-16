
(define-module (opencog nlp relex2logic))

; (add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog atom-types) (opencog nlp))

(load "relex2logic/rule-utils.scm")
(load "relex2logic/r2l-utilities.scm")
(load "relex2logic/tv-utilities.scm")
(load "relex2logic/post-processing.scm")

; -----------------------------------------------------------------------
; This loads all the rules into the cogserver shell.
(define-public (load-r2l-rulebase)

	; "." in case the cogserver is started from in-source build directory.
	(add-to-load-path ".")
	(load "relex2logic/rule-utils.scm")  ; XXX
	(load "relex2logic/rule-helpers.scm")
	(load "relex2logic/loader/load-rules.scm")  ; XXX
	(load "relex2logic/loader/gen-r2l-en-rulebase.scm")

	*unspecified*  ; no return value, avoids printing gunk.
)

(load-r2l-rulebase)
