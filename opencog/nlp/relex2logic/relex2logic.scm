
(define-module (opencog nlp relex2logic))

; (add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog atom-types) (opencog nlp))

(load "relex2logic/rule-utils.scm")
(load "relex2logic/r2l-utilities.scm")

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

; XXX TODO FIXME Currently, the statement below fails to have the
; expected effect: although it loads the rules, it loads them where
; the URE can't find them.  Its actually a URE bug, the URE is looking
; for rules in the wrong places. See bug
; https://github.com/opencog/opencog/issues/2021
; for the tracking status.
(load-r2l-rulebase)
