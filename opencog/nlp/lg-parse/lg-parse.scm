;
; lg-parse.scm
;
; Link Grammar Parsing API
;
(define-module (opencog nlp lg-parse))

(use-modules (opencog) (opencog oc-config) (opencog nlp))

(load-extension (string-append opencog-ext-path-lg-parse "liblg-parse") "opencog_nlp_lgparse_init")
