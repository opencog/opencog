;; 3. If and-BIT A expands into and-BIT B that is a preproof of target T,
;;    then A is a preproof of T as well.
;;
;; ImplicationScope <1 1>
;;   VariableList
;;     TypedVariable
;;       Variable "$A"
;;       Type "BindLink"
;;     TypedVariable
;;       Variable "$B"
;;       Type "BindLink"
;;     TypedVariable
;;       Variable "$R"
;;       Type "BindLink"
;;     Variable "$L"
;;     Variable "$T"
;;   And
;;     Execution
;;       Schema "URE:BC:expand-and-BIT"
;;       List
;;         Variable "$A"
;;         Variable "$L"
;;         Variable "$R"
;;       Variable "$B"
;;     Evaluation
;;       Predicate "URE:BC:preproof"
;;       List
;;         Variable "$B"
;;         Variable "$T"
;;   Evaluation
;;     Predicate "URE:BC:preproof"
;;     List
;;       Variable "$A"
;;       Variable "$T"
;;
;; We could add more knowledge such that initial and-BITs like
;;
;; Bind
;;   T
;;   T
;;
;; are preproof if there exists a proof of T but it doesn't appear
;; immediately obviously useful so we let that aside.
;;
;; Technical remark: and-BITs, rules are wrapped by DontExecLinks to
;; garanty that they won't be executed while reasoning on them.
(define preproof-expander-is-preproof
  (ImplicationScope (stv 1 1)
    (VariableList
      (TypedVariable
        (Variable "$A")
        (Type "DontExecLink"))
      (TypedVariable
        (Variable "$B")
        (Type "DontExecLink"))
      (TypedVariable
        (Variable "$R")
          (Type "DontExecLink"))
      (Variable "$L")
      (Variable "$T"))
    (And
      (Execution
        (Schema "URE:BC:expand-and-BIT")
        (List
          (Variable "$A")
          (Variable "$L")
          (Variable "$R"))
        (Variable "$B"))
      (Evaluation
        (Predicate "URE:BC:preproof")
        (List
          (Variable "$B")
          (Variable "$T"))))
    (Evaluation
      (Predicate "URE:BC:preproof")
      (List
        (Variable "$A")
        (Variable "$T")))))

;; Load useful PLN rules
(define rule-filenames
  (list "meta-rules/predicate/conditional-full-instantiation.scm"
        "rules/propositional/fuzzy-conjunction-introduction.scm"
        )
  )
(for-each load-from-path rule-filenames)

;; Turn preproof-expander-is-preproof into a rule
(define preproof-expander-is-preproof-rule
  (car (apply-rule conditional-full-instantiation-implication-scope-meta-rule
                   preproof-expander-is-preproof)))
(define preproof-expander-is-preproof-rule-name
  (DefinedSchemaNode "preproof-expander-is-preproof-rule"))
(DefineLink preproof-expander-is-preproof-rule-name
  preproof-expander-is-preproof-rule)

;; Define rule base
(define pep-rbs (ConceptNode "preproof-expander-is-preproof-rule-base"))

;; Define ppc-bc for convenience
(define (pep-bc . args)
  (apply cog-bc (cons pep-rbs args)))

;; List the rules
(define rules
  (list
     preproof-expander-is-preproof-rule-name
     fuzzy-conjunction-introduction-2ary-rule-name
  )
)

;; Associate rules to rule base
(ure-add-rules pep-rbs rules)

;; Termination criteria parameters. We need 2*piter + 1, 1 to
;; initialize the BIT, and 2*piter because piter is the maximum depth
;; of expansions that may happen while solving the problem, and we
;; need to combine 2 rules, preproof-expander-is-preproof and
;; fuzzy-conjunction-introduction-2ary.
(ure-set-num-parameter pep-rbs "URE:maximum-iterations" (+ 1 (* 2 piter)))
