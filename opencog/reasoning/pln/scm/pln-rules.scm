;
; Scheme helper functions to apply pln rules
;
; Author Nil Geisweiller <ngeiswei@gmail.com>
;

; rules that do not require preprocessing
(define (SubsetEvalRule sub super . contexts)
  (pln-ar "SubsetEvalRule" (list sub super) contexts))
(define (IntInhRule sub super . contexts)
  (pln-ar "IntensionalInheritanceRule" (list sub super) contexts))
(define (InhDedRule AB BC . contexts)
  (pln-ar "InheritanceDeductionRule" (list AB BC) contexts))
; context rules, not sure if the last argument 'contexts' would be useful
(define (ContextualizerRule R . contexts)
  (pln-ar "ContextualizerRule" (list R) contexts))
(define (DecontextualizerRule CL . contexts)
  (pln-ar "DecontextualizerRule" (list CL) contexts))
(define (ContextFreeToSensitiveRule CX A . contexts)
  (pln-ar "ContextFreeToSensitiveRule" (list CX A) contexts))

; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedent . contexts)
  (pln-ar "ModusPonensRule" (list implication antecedent) contexts))

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
(define (ForAllInstantiationRuleNameStr forAll)
  (string-append "ForAllInstantiationRule"
                 (number->string (cog-handle forAll))))
; apply the inference rule
(define (ForAllInstantiationRule forAll . arguments)
  (pln-ar (ForAllInstantiationRuleNameStr forAll)
          (list (universal-instantiate forAll arguments))
          (list))) ; TODO
