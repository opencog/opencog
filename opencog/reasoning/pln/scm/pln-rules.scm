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

; Deduction rules
(define (InheritanceDeductionRule AB BC . contexts)
  (pln-ar "InheritanceDeductionRule" (list AB BC) contexts))
(define (SubsetDeductionRule AB BC . contexts)
  (pln-ar "SubsetDeductionRule" (list AB BC) contexts))

; Logical rules
(define (NotRule A . contexts)
  (pln-ar "NotRule" (list A) contexts))
; And rules, must happend the arity to the rule name
(define (SimpleAndRuleNameStr premises)
  (string-append "SimpleAndRule" (number->string (length premises))))
(define (SimpleAndRule . premises)
  (pln-ar (SimpleAndRuleNameStr premises) premises (list))) ;TODO add
                                                            ;context
; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedent . contexts)
  (pln-ar "ModusPonensRule" (list implication antecedent) contexts))

; Substitution rules
; that one substitute the right operand or R by the left one in C
(define (InheritanceSubstRule R C . contexts)
  (pln-ar "InheritanceSubstRByLRule" (list R C) contexts))

; Context rules, not sure if the last argument 'contexts' would be useful
(define (ContextualizerRule R . contexts)
  (pln-ar "ContextualizerRule" (list R) contexts))
(define (DecontextualizerRule CL . contexts)
  (pln-ar "DecontextualizerRule" (list CL) contexts))
(define (ContextFreeToSensitiveRule CX A . contexts)
  (pln-ar "ContextFreeToSensitiveRule" (list CX A) contexts))

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
