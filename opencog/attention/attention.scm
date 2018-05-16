;
; Opencog attention module
;

(use-modules (opencog))  ; needed for cog-type->int and LTDL

(define-module (opencog attention))

; Load the C library that calls the classserver to load the types.
(load-extension "libattention-types" "attention_types_init")

(load "attention/attention_types.scm")
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
