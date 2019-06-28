;; =============================================================================
;; ModusPonensRule
;; 
;; <LinkType>
;;   A
;;   B
;; A
;; |-
;; B
;;
;; Due to type system limitations, the rule has been divided into 3:
;;       modus-ponens-inheritance-rule
;;       modus-ponens-implication-rule
;;       modus-ponens-subset-rule
;;
;; This rule contains less premises but thus is less precises than
;; PreciseModusPonens. In order to properly calculate P(B) it must
;; account for P(B|A) and P(B|Not A), as follows
;;
;; P(B) = P(B|A)*P(A) + P(B|Not A)*P(Not A)
;;
;; However in this rule to avoid requiring too many premises it is
;; assumed
;;
;; P(B|Not A) = 0.2
;;
;; which is a temporary hack. Maybe the resulting confidence could at
;; least be degraded to conpensate for that hack. But ideally one
;; should use a precise modus ponens (requiring P(B|Not A) as
;; premises), and then if missing, these premises could be estimated
;; via other rules.
;;
;; Precise modus ponens rule is implemented in
;;
;; opencog/pln/rules/wip/precise-modus-ponens.scm
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

;; Generate the corresponding modus ponens rule given its link-type.
(define (gen-modus-ponens-rule link-type)
  (let* ((A (Variable "$A"))
         (B (Variable "$B"))
         (AB (link-type
               A
               B)))
  (Bind
    ;; Variable declaration
    (VariableList
      A
      B)
    ;; Patterns
    (And
      ;; Preconditions
      (Evaluation
        (GroundedPredicate "scm: gt-zero-confidence")
        A)
      (Evaluation
        (GroundedPredicate "scm: gt-zero-confidence")
        AB)
      ;; Pattern clauses
      (Present
        AB
        A))
    ;; Rewrite
    (ExecutionOutputLink
      (GroundedSchemaNode "scm: modus-ponens-formula")
      (ListLink
        B
        AB
        A)))))

(define modus-ponens-inheritance-rule
  (gen-modus-ponens-rule InheritanceLink))

(define modus-ponens-implication-rule
  (gen-modus-ponens-rule ImplicationLink))

(define modus-ponens-subset-rule
  (gen-modus-ponens-rule SubsetLink))

(define (modus-ponens-formula B AB A)
  (let
      ((sA (cog-mean A))
       (cA (cog-confidence A))
       (sAB (cog-mean AB))
       (cAB (cog-confidence AB))
       (snotAB 0.2)                     ; Huge hack
       (cnotAB 1))
    (cog-set-tv!
     B
     (stv 
      (precise-modus-ponens-strength-formula sA sAB snotAB) 
      (min (min cAB cnotAB) cA)))))

;; Name the rules
(define modus-ponens-inheritance-rule-name
  (DefinedSchemaNode "modus-ponens-inheritance-rule"))
(DefineLink modus-ponens-inheritance-rule-name
  modus-ponens-inheritance-rule)

(define modus-ponens-implication-rule-name
  (DefinedSchemaNode "modus-ponens-implication-rule"))
(DefineLink modus-ponens-implication-rule-name
  modus-ponens-implication-rule)

(define modus-ponens-subset-rule-name
  (DefinedSchemaNode "modus-ponens-subset-rule"))
(DefineLink modus-ponens-subset-rule-name
  modus-ponens-subset-rule)
