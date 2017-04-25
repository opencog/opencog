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

(load "formulas.scm")

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
        (Variable "$C")
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
        (Implication
           (And
              (Variable "$A")
              (Variable "$B"))
           (Variable "$C"))
        (VariableNode "$A")
        (VariableNode "$B")
        (VariableNode "$C")
        (ImplicationLink
           (VariableNode "$A")
           (VariableNode "$C"))
        (ImplicationLink
           (VariableNode "$B")
           (VariableNode "$C")))))

(define implication-implicant-conjunction-rule
  (Bind
     implication-implicant-conjunction-variables
     implication-implicant-conjunction-body
     implication-implicant-conjunction-rewrite))

;; Computing the strength is based on
;;
;;    P(C|A) = P(C,A)/P(A)
;;    P(C|B) = P(C,B)/P(B)
;;
;; By Bayes rule
;;
;;    P(C|A,B) = P(A,B|C) * P(C) / P(A,B)
;;
;; Let's assume that A and B are independent, as well as independent
;; when conditioned by C, that is
;;
;;    P(A,B) = P(A) * P(B)
;;    P(A,B|C) = P(A|C) * P(B|C)
;;
;; Thus
;;
;;    P(C|A,B) = P(A|C) * P(B|C) * P(C) / (P(A) * P(B))
;;
;; By Bayes rule
;;
;;    P(A|C) = P(C|A) * P(A) / P(C)
;;    P(B|C) = P(C|B) * P(B) / P(C)
;;
;; Thus
;;
;;    P(C|A,B) = P(C|A) * P(C|B) / P(C)
;;
;; TODO: there is something weird, if P(C) is tiny then P(C|A,B) goes
;; above 1, we need to understand why. Meanwhile, we just cap at 1.
;;
;; TODO: A, B and AC, BC could be wrapped in SetLinks (to speed up the
;; BC).
(define (implication-implicant-conjunction-formula ABC A B C AC BC)
  (let* 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (sC (cog-stv-strength C))
       (sAC (cog-stv-strength AC))
       (sBC (cog-stv-strength BC))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B))
       (cC (cog-stv-confidence C))
       (cAC (cog-stv-confidence AC))
       (cBC (cog-stv-confidence BC)))
    (if
       (and
           (conditional-probability-consistency sA sC sAC)
           (conditional-probability-consistency sB sC sBC))
       ;; Consistency is met build the resulting implication
       (let ((AnB (gar ABC)))
         (cog-set-tv! AnB (stv (* sA sB) (min cA cB)))
         (cog-set-tv!
            ABC
            (stv (implication-implicant-conjunction-strength sC sAC sBC)
                 (implication-implicant-conjunction-confidence cAC cBC))))
       ;; Inconsistent, better not build anything
       (cog-undefined-handle))))

(define (implication-implicant-conjunction-strength sC sAC sBC)
  (min (/ (* sAC sBC) sC) 1))

(define (implication-implicant-conjunction-confidence cAC cBC)
  (min cAC cBC))

; Name the rule
(define implication-implicant-conjunction-rule-name
  (DefinedSchemaNode "implication-implicant-conjunction-rule"))
(DefineLink implication-implicant-conjunction-rule-name
  implication-implicant-conjunction-rule)
