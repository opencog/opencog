;
; Scheme helper functions to apply pln rules
;
; Author Nil Geisweiller <ngeiswei@gmail.com>
;

; rules that do not require preprocessing
(define (SubsetEvalRule sub super)
  (pln-ar "SubsetEvalRule" (list sub super)))
(define (IntInhRule sub super)
  (pln-ar "IntensionalInheritanceRule" (list sub super)))
; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedent)
  (pln-ar "ModusPonensRule" (list implication antecedent)))

; AND rules, must happend the arity to the rule name
(define (SimpleANDRuleNameStr premises)
  (string-append "SimpleANDRule" (number->string (length premises))))
(define (SimpleANDRule . premises)
  (pln-ar (SimpleANDRuleNameStr premises) premises))

;
; UniversalInstantiationForAllRule
;
; premises are as follows
; premises[0] is the forall atom = (ForAllLink (ListLink X1,...,Xn) Body)
; premises[1:n] are the arguments to substitute to X1,...,Xn
;
; It generates the instance and then uses CustomCrispUnificationRule
; to compute its TV.

; define rule name 
(define (UniversalInstantiationForAllRuleNameStr forAll)
  (string-append "CustomCrispUnificationRule"
                 (number->string (cog-handle forAll))))
; apply the inference rule
(define (UniversalInstantiationForAllRule forAll . arguments)
  (pln-ar (UniversalInstantiationForAllRuleNameStr forAll)
          (list (universal-instantiate forAll arguments))))
