;
; Opencog attention module
;

(use-modules (opencog))

(define-module (opencog attention))

(load "attention/default-param-values.scm")

(define-public (ecan-set-spreading-filter . type-symbols)
"
  ecan-set-spreading-filter TYPE-SYMBOLS

  Set ecan to filter atoms of TYPE-SYMOBLS.
"
  (if (not (null? type-symbols))
    (StateLink
      SPREADING_FILTER
      (MemberLink
        (map (lambda (x) (TypeNode (symbol->string x))) type-symbols))))

  SPREADING_FILTER
)
