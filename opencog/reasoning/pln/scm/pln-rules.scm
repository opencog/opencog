;
; Scheme helper functions to apply pln rules
;
(define (SubsetEvalRule sub super)
  (pln-ar "SubsetEvalRule" (list sub super)))
(define (IntInhRule sub super)
  (pln-ar "IntensionalInheritanceRule" (list sub super)))
; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedant)
  (pln-ar "ModusPonensRule" (list implication antecedant)))
; AND rules
(define (SimpleANDRuleNameStr premises)
  (string-append "SimpleANDRule" (number->string (length premises))))
(define (SimpleANDRule . premises)
  (pln-ar (SimpleANDRuleNameStr premises) premises))
