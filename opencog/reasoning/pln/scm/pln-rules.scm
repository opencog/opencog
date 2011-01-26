;
; Scheme helper functions to apply pln rules
;
; Author Nil Geisweiller <ngeiswei@gmail.com>
;

; rules that do not require preprocessing
(define (SubsetEvalRule sub super . contexts)
  (pln-ar "SubsetEvalRule" (list sub super) contexts))
(define (IntensionalInheritanceRule sub super . contexts)
  (pln-ar "IntensionalInheritanceRule" (list sub super) contexts))
(define (InhDeductionRule AB BC . contexts)
  (pln-ar "InheritanceDeductionRule" (list AB BC) contexts))
(define (SubsetDeductionRule AB BC . contexts)
  (pln-ar "SubsetDeductionRule" (list AB BC) contexts))
(define (NotRule A . contexts)
  (pln-ar "NotRule" (list A) contexts))
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
; Instantiation Rules
;
; premises are as follows
; premises[0] is the forall atom = (Quantifier (ListLink X1,...,Xn) Body)
; premises[1:n] are the arguments to substitute to X1,...,Xn
;
; It generates the instance and then uses an Instantiation rule
; to compute its TV.

; define rule name
(define (append-handle s a)
  (string-append s (number->string (cog-handle a))))
(define (ForAllInstantiationRuleNameStr forAll)
  (append-handle "ForAllInstantiationRule" forAll))
(define (AverageInstantiationRuleNameStr average)
  (append-handle "AverageInstantiationRule" average))
; apply the inference rule
(define (ForAllInstantiationRule forAll . arguments)
  (pln-ar (ForAllInstantiationRuleNameStr forAll)
          (list (universal-instantiate forAll arguments))
          (list))) ; TODO context list
(define (AverageInstantiationRule average . arguments)
  (pln-ar (AverageInstantiationRuleNameStr average)
          (list (universal-instantiate average arguments))
          (list))) ; TODO context list
