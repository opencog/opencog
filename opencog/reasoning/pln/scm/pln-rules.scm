;
; Scheme helper functions to apply pln rules
;

; rules that do not require preprocessing
(define (SubsetEvalRule sub super)
  (pln-ar "SubsetEvalRule" (list sub super)))
(define (IntInhRule sub super)
  (pln-ar "IntensionalInheritanceRule" (list sub super)))
; ModusPonensRule is the name for StrictImplicationBreakdownRule
(define (ModusPonensRule implication antecedant)
  (pln-ar "ModusPonensRule" (list implication antecedant)))

; AND rules, must happend the arity to the rule name
(define (SimpleANDRuleNameStr premises)
  (string-append "SimpleANDRule" (number->string (length premises))))
(define (SimpleANDRule . premises)
  (pln-ar (SimpleANDRuleNameStr premises) premises))

;
; CustomCrispForAllRule
;
; premises are as follows
; premises[0] is the forall atom = (ForAllLink (ListLink X1,...,Xn) Body)
; premises[1:n] are the arguments to substitute to X1,...,Xn
;
; It generates the instance and then uses CustomCrispUnificationRule
; to compute its TV.

; define rule name 
(define (CustomCrispForAllRuleNameStr premises)
  (string-append "CustomCrispUnificationRule" 
                 (number->string (cog-handle (car premises)))))
; turn the premises into a list of pairs (variable, atoms)
(define (extract-bindings premises)
  (let* ((forAll (car premises)) (arguments (cdr premises))
         (variables (cog-outgoing-set (car (cog-outgoing-set forAll)))))
    (map (lambda (v a) (cons v a)) variables arguments)))
; extract the body from the premises
(define (extract-body premises)
  (car (cdr (cog-outgoing-set (car premises)))))
; take input 1) a list of pairs (variable, atoms) containing the
; variables and their corresponding arguments and 2) the atom root of
; the body and return the atom of the body where all variables have
; been substituted by the arguments
(define (InstantiateBody_rec bindings body)
  (let ((body_type (cog-type body)))
    (cond ; recursive case
          ((cog-subtype? 'Link body_type)
           (apply cog-new-link 
                  (cons body_type
                        (map (lambda (child) (InstantiateBody_rec bindings child)) 
                             (cog-outgoing-set body)))))
          ; base cases
          ((cog-subtype? 'VariableNode body_type) (cdr (assoc body bindings)))
          ((cog-subtype? 'Node body_type) body))))
; perform universal instantiation given the ForAll and the arguments
(define (InstantiateBody premises)
  (InstantiateBody_rec (extract-bindings premises) (extract-body premises)))
; apply the inference rule
(define (CustomCrispForAllRule . premises)
  (pln-ar (CustomCrispForAllRuleNameStr premises)
          (list (InstantiateBody premises))))
