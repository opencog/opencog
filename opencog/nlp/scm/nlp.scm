
(define-module (opencog nlp))

(use-modules (opencog) (opencog atom-types))

; XXX WTF FIXME
; When starting the cogserver (but not from the guile repl prompt!)
; I get the error:
; "In procedure module-lookup: Unbound variable: stv"
; "In procedure module-lookup: Unbound variable: cog-chase-link"
; unless I *explcitly* load "utilities.scm" below. But that's crazy!!
; The (use-modules (opencog)) above should be enough to avoid this.
; The actual bugs show up in relex2logic, sureal and/or microplanning.
; There might be a weird mircorplanning bug ...
; Something is really messed up with how the cogserver is handling
; modules...
(load-from-path "utilities.scm")

; Load various parts....
(load "nlp/nlp-utils.scm")
(load "nlp/disjunct-list.scm")
