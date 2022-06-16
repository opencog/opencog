(use-modules (opencog) (opencog nlp))

(define-module (opencog nlp oc)
  #:use-module (opencog)
  #:use-module (opencog nlp)
  #:use-module (opencog oc-config)
)

; Load the C library; this calls the nameserver to load the types.
(load-extension (string-append opencog-ext-path-nlp-oc-types "libnlp-oc-types") "nlp_oc_types_init")

; Load various parts....
(include-from-path "opencog/nlp/oc/nlp_oc_types.scm")
; NOTE: relex-utils.scm is used by cmake for configuring relex dependent tests.
; Update relevant paths should you move it.
(include-from-path "opencog/nlp/oc/relex-utils.scm")
(include-from-path "opencog/nlp/oc/processing-utils.scm")

; Weird ... MUST say `(export)` or no define-publics are visible!
; XXX What? nothing else anywhere needs this! FIXME, somethings broke.
(export)
