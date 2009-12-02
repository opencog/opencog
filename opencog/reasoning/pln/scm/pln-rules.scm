; Scheme helper functions to apply pln rules
(define (SubsetEvalRule sub super)
  (pln-ar "SubsetEvalRule" (list sub super)))
(define (IntInhRule sub super)
  (pln-ar "IntensionalInheritanceRule" (list sub super)))
; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedant)
  (pln-ar "ModusPonensRule" (list implication antecedant)))
