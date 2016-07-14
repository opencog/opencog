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
(use-modules (opencog logger))

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
     ;; Current hack to limit X as concepts
     (TypedVariable
        (Variable "$X")
        (Type "ConceptNode"))))

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

;; Return #t is the strength of the TV of A is above 0.5 and its
;; confidence is above zero.
(define (true-enough? A)
  (let* (
         (TV (cog-tv A))
         (s (tv-mean TV))
         (c (tv-conf TV)))
    (and (> s 0.5) (> c 0))))

(define (implication-direct-evaluation-formula P Q)
  (let* (
         (K 800) ; parameter to convert from count to confidence
         ;; Current hack to limit X as concepts
         (X (Variable "$X"))
         (vardecl (TypedVariable X (Type "ConceptNode")))
         (term->instance (lambda (p x) (Evaluation p x)))
         (true-enough-term? (lambda (p x) (true-enough? (term->instance p x))))
         (fetch-true-enough-terms
          (lambda (p)
            (let* ((query (Get vardecl (term->instance p X)))
                   (terms (cog-outgoing-set (cog-satisfying-set query))))
              (filter (lambda (x) (true-enough-term? p x)) terms))))
         (P-true-enough-terms (fetch-true-enough-terms P))
         (Q-true-enough-terms (fetch-true-enough-terms Q))
         (P-length (length P-true-enough-terms))
         (P-inter-Q-terms (lset-intersection equal?
                                             P-true-enough-terms
                                             Q-true-enough-terms))
         (P-inter-Q-length (length P-inter-Q-terms))
         (TV-strength (if (> P-length 0)
                          (exact->inexact (/ P-inter-Q-length P-length))
                          0))
         (TV-confidence (exact->inexact (/ P-length K))))
    ;; (cog-logger-debug "[PLN-Induction] P = ~a" P)
    ;; (cog-logger-debug "[PLN-Induction] Q = ~a" Q)
    ;; (cog-logger-debug "[PLN-Induction] P-true-enough-terms = ~a" P-true-enough-terms) 
    ;; (cog-logger-debug "[PLN-Induction] Q-true-enough-terms = ~a" Q-true-enough-terms)
    ;; (cog-logger-debug "[PLN-Induction] P-length = ~a" P-length)
    ;; (cog-logger-debug "[PLN-Induction] P-inter-Q-length = ~a" P-inter-Q-length)
    ;; (cog-logger-debug "[PLN-Induction] TV-strength = ~a" TV-strength)
    ;; (cog-logger-debug "[PLN-Induction] TV-confidence = ~a" TV-confidence)
    (if (> TV-confidence 0)
        (Implication (stv TV-strength TV-confidence) P Q)
        (cog-undefined-handle))))

;; Name the rule
(define implication-direct-evaluation-rule-name
  (DefinedSchemaNode "implication-direct-evaluation-rule"))
(DefineLink implication-direct-evaluation-rule-name
  implication-direct-evaluation-rule)
