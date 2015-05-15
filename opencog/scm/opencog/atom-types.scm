;
; Opencog atom-types module
;
(define-module (opencog atom-types))

; Alternately, we could also have
; (define-module (opencog atomtypes nlp-types))
; (define-module (opencog atomtypes spacetime-types))
; and so on, but I don't see the point of that, at the moment...

(load-from-path "nlp/types/nlp_types.scm")
(load-from-path "spacetime/spacetime_types.scm")
(load-from-path "dynamics/attention/attention_types.scm")
(load-from-path "embodiment/embodiment_types.scm")
