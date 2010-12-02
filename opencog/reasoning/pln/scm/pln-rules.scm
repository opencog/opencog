;
; Scheme helper functions to apply pln rules
;
; Author Nil Geisweiller <ngeiswei@gmail.com>
;

; rules that do not require preprocessing
(define (SubsetEvalRule sub super . context)
  (pln-ar "SubsetEvalRule" (list sub super) context))
(define (IntInhRule sub super . context)
  (pln-ar "IntensionalInheritanceRule" (list sub super) context))
(define (InhDedRule AB BC . context)
  (pln-ar "InheritanceDeductionRule" (list AB BC) context))

; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedent . context)
  (pln-ar "ModusPonensRule" (list implication antecedent) context))

; AND rules, must happend the arity to the rule name
(define (SimpleANDRuleNameStr premises)
  (string-append "SimpleANDRule" (number->string (length premises))))
(define (SimpleANDRule . premises)
  (pln-ar (SimpleANDRuleNameStr premises) premises (list))) ;TODO

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
(define (UniInsForAllRuleNameStr forAll)
  (string-append "CustomCrispUnificationRule"
                 (number->string (cog-handle forAll))))
; apply the inference rule
(define (UniInsForAllRule forAll . arguments)
  (pln-ar (UniInsForAllRuleNameStr forAll)
          (list (universal-instantiate forAll arguments))
          (list))) ; TODO
