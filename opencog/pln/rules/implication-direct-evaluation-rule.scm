;; =====================================================================
;; Implication direct evaluation rule
;;
;; Evaluation
;;   P
;;   X1
;; ...
;; Evaluation
;;   P
;;   Xn
;; Evaluation
;;   Q
;;   Xn+1
;; ...
;; Evaluation
;;   Q
;;   Xm
;; |-
;; Implication <TV>
;;    P
;;    Q
;;
;; where the TV strength and the count are calculated based on the
;; instances P(X1), ..., P(Xn), Q(Xn+1), ..., Q(Xm).
;;
;; ----------------------------------------------------------------------

(use-modules (srfi srfi-1))

;; Rather than building as many rules as n and m we build the
;; following one
;;
;; Evaluation
;;   P
;;   X
;; Evaluation
;;   Q
;;   X
;; |-
;; Implication
;;   P
;;   Q
;;
;; and retrieve all the other instances in the rule formula

(define implication-direct-evaluation-vardecl
  (VariableList
     (TypedVariable
        (Variable "$P")
        (Type "PredicateNode"))
     (TypedVariable
        (Variable "$Q")
        (Type "PredicateNode"))
     (Variable "$X")))

(define implication-direct-evaluation-pattern
  (And
     (Evaluation
        (Variable "$P")
        (Variable "$X"))
     (Evaluation
        (Variable "$Q")
        (Variable "$X"))
     (Not
        (Equal
           (Variable "$P")
           (Variable "$Q")))))

(define implication-direct-evaluation-rewrite
  (ExecutionOutput
     (GroundedSchema "scm: implication-direct-evaluation-formula")
     (List
        (Variable "$P")
        (Variable "$Q"))))

(define implication-direct-evaluation-rule
  (Bind
     implication-direct-evaluation-vardecl
     implication-direct-evaluation-pattern
     implication-direct-evaluation-rewrite))

(define (implication-direct-evaluation-formula P Q)
  (let* (
         (K 800) ; parameter to convert from count to confidence
         (P-query (Get (Evaluation P (Variable "$X"))))
         (P-instances (cog-outgoing-set (cog-satisfying-set P-query)))
         (P-length (length P-instances))
         (Q-query (Get (Evaluation Q (Variable "$X"))))
         (Q-instances (cog-outgoing-set (cog-satisfying-set Q-query)))
         (P-inter-Q-instances (lset-intersection equal?
                                                 P-instances
                                                 Q-instances))
         (P-inter-Q-length (length P-inter-Q-instances))
         (TV-strength (exact->inexact (/ P-inter-Q-length P-length)))
         (TV-confidence (exact->inexact (/ P-length K))))
    (Implication (stv TV-strength TV-confidence) P Q)))

;; Name the rule
(define implication-direct-evaluation-rule-name
  (DefinedSchemaNode "implication-direct-evaluation-rule"))
(DefineLink implication-direct-evaluation-rule-name
  implication-direct-evaluation-rule)
