(use-modules (opencog))

(define-module (opencog nlp))

; Load the C library; this calls the nameserver to load the types.
(load-extension "libnlp-types" "nlp_types_init")

; User-modifiable config parameters.
; We'll keep these here for backwards-compat, for now, but it is
; recommended that (use-relex-server HOST PORT) be used instead...
(define-public relex-server-host "127.0.0.1")
(define-public relex-server-port 4444)

; Load various parts....
(load "nlp/types/nlp_types.scm")
(load "nlp/nlp-utils.scm")
(load "nlp/relex-utils.scm")
(load "nlp/processing-utils.scm")

; Weird ... MUST say `(export)` or no define-publics are visible!
; XXX What? nothing else anywhere needs this! FIXME, somethings broke.
(export)
