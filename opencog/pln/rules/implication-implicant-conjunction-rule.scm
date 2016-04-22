;; =====================================================================
;; ImplicationImplicantConjunctionRule
;;
;; ImplicationLink <TV1>
;;    A
;;    C
;; ImplicationLink <TV2>
;;    B
;;    C
;; |-
;; ImplicationLink <TV>
;;    AndLink
;;       A
;;       B
;;    C
;;----------------------------------------------------------------------

(define implication-implicant-conjunction-variables
  (VariableList
     (TypedVariable
        (Variable "$A")
        (TypeChoice
           (Type "PredicateNode")
           (Type "LambdaLink")))
     (TypedVariable
        (Variable "$B")
        (TypeChoice
           (Type "PredicateNode")
           (Type "LambdaLink")))
     (TypedVariable
        (Variable "$B")
        (TypeChoice
           (Type "PredicateNode")
           (Type "LambdaLink")))))

(define implication-implicant-conjunction-body
  (AndLink
     (ImplicationLink
        (VariableNode "$A")
        (VariableNode "$C"))
     (ImplicationLink
        (VariableNode "$B")
        (VariableNode "$C"))
     (NotLink (EqualLink (VariableNode "$A") (VariableNode "$B")))))

(define implication-implicant-conjunction-rewrite
  (ExecutionOutput
     (GroundedSchema "scm: implication-implicant-conjunction-formula")
     (List
        (VariableNode "$C")
        (ImplicationLink
           (VariableNode "$A")
           (VariableNode "$C"))
        (ImplicationLink
           (VariableNode "$B")
           (VariableNode "$C"))
        (Implication
           (And
              (Variable "$A")
              (Variable "$B"))
           (Variable "$C")))))

(define implication-implicant-conjunction-rule
  (Bind
     implication-implicant-conjunction-variables
     implication-implicant-conjunction-body
     implication-implicant-conjunction-rewrite))

(define (implication-implicant-conjunction-formula C AC BC ABC)
  (cog-set-tv! ABC
   (implication-implicant-conjunction-side-effect-free-formula C AC BC)))

;; Computing the strength is based on
;;
;; P(C|A) = P(C,A)/P(A)
;; P(C|B) = P(C,B)/P(B)
;;
;; Let's assume that
;;
;; P(A,B) = P(A)*P(B)
;;
;; Thus
;;
;; P(C|A,B) = P(C,A,B)/P(A,B)
;; = P(C,A,B) / (P(A)*P(B))
;;
;; Let's assume that P(A,B|C) = P(A|C) * P(B|C), then we can write
;;
;; P(A,B,C)/P(C) = P(A,C)/P(C) * P(B,C)/P(C)
;; P(A,B,C) = P(A,C) * P(B,C) / P(C)
;;
;; P(C|A,B) = (P(A,C) * P(B,C) / P(C)) / (P(A)*P(B))
;;
;; P(C|A,B) = P(C|A)*P(C|B)/P(C)
;;
(define (implication-implicant-conjunction-side-effect-free-formula C AC BC)
  (let* 
      ((sC (cog-stv-strength C))
       (sAC (cog-stv-strength AC))
       (sBC (cog-stv-strength BC))
       (cAC (cog-stv-confidence AC))
       (cBC (cog-stv-confidence BC)))
    (stv (/ (* sAC sBC) sC)
         (min cAC cBC))))

; Name the rule
(define implication-implicant-conjunction-rule-name
  (DefinedSchemaNode "implication-implicant-conjunction-rule"))
(DefineLink implication-implicant-conjunction-rule-name
  implication-implicant-conjunction-rule)
