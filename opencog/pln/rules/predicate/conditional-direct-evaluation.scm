;; Conditional Direct Evaluation Rule
;;
;; I
;; |-
;; I <TV>
;;
;; Calculate the TV of I based on direct evidence. I is a conditional,
;; like an ImplicationLink, ImplicationScopeLink, InheritanceLink,
;; etc.
;;
;; TODO: we should make the evidence as premises. One way to do that
;; would be to calculate incrementally, keeping track of all evidence
;; that have been used to calculate its TV and choose one that hasn't
;; so been used so far. This would allow to evaluate evidence if
;; necessary (if they are for instance conjunctions of other things we
;; know, we still need to calculate the conjunctions).

(use-modules (srfi srfi-1))
(use-modules (opencog query))

;; TODO: turn that into a generator

(define conditional-direct-evaluation-implication-scope-rule
  (Bind
    (TypedVariable
      (Variable "$I")
      (Type "ImplicationScopeLink"))
    (Variable "$I")
    (ExecutionOutput
      (GroundedSchema "scm: conditional-direct-evaluation-implication-scope-formula")
      (Variable "$I"))))

(define (conditional-direct-evaluation-implication-scope-formula I)
  (let* ((out (cog-outgoing-set I))
         (arity (length out))
         (vardecl (if (= arity 2) #f (list-ref out 0)))
         (antecedent (list-ref out (if (= arity 2) 0 1)))
         (consequent (list-ref out (if (= arity 2) 1 2)))
         ;; Fetch all antecedent terms, or rather their values
         ;; associated to their variables
         (query (Get vardecl antecedent))
         (antecedent-values (cog-satisfying-set query))
         ;; Generate the antecedent and consequent terms, to get there
         ;; TVs
         (antecedent-terms (Put vardecl antecedent (List antecedent-values)))
         (consequent-terms (Put vardecl consequent (List consequent-values)))
         ;; Calculate the TV based on the evidence
         (tv (evidence-to-tv antecedent-terms consequent-terms)))
    (if (tv-non-null-conf? tv)
        (I tv))))

(define (evidence-to-tv antecedent-terms consequent-terms)
  (let* ;; TODO replace by a distributional TV based calculation.
      ((K 800) ; parameter to convert from count to confidence
       (true-enough? (lambda A) (let* ((TV (cog-tv A))
                                       (s (tv-mean TV))
                                       (c (tv-conf TV)))
                                  (and (> s 0.5) (> c 0))))
       (both-true-enough? (lambda A B) (and (true-enough? A) (true-enough? B)))
       (true-enough-antecedent-terms (filter true-enough? antecedent-terms))
       (true-enough-inter-terms (filter both true-enough?
                                        antecedent-terms consequent-terms))
       (antecedent-length (length true-enough-antecedent-terms))
       (inter-length (length true-enough-inter-terms))
       (strength (if (> antecedent-length 0)
                        (exact->inexact (/ inter-length antecedent-length))
                        0))
       (confidence (exact->inexact (/ antecedent-length K))))
    (stv strength confidence)))

;; Name the rule
(define conditional-direct-evaluation-implication-scope-rule-name
  (DefinedSchemaNode "conditional-direct-evaluation-implication-scope-rule"))
(DefineLink conditional-direct-evaluation-implication-scope-rule-name
  conditional-direct-evaluation-implication-scope-rule)
