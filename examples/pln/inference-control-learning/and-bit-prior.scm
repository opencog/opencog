;; 1. Any and-BIT has a very low probability, 0.0001%, of being the
;;    preproof of any target, and we have a very low confidence, 0.01,
;;    about this probability. These numbers are made up. This should
;;    be replaced by a set of more informative laws about the space of
;;    proofs, so that we can for instance infer that small and-BITs
;;    are more likely to be preproofs than larger and-BITs, etc.
;;
;; ImplicationScope <0.0001 0.01>
;;   VariableList
;;     TypedVariable
;;       Variable "$A"
;;       Type "BindLink"
;;     Variable "$T"
;;   And
;;     Evaluation
;;       Predicate "URE:BC:and-BIT"
;;       Variable "$A"
;;     Evaluation
;;       Predicate "URE:BC:target"
;;       Variable "$T"
;;   Evaluation
;;     Predicate "URE:BC:preproof"
;;     List
;;       Variable "$A"
;;       Variable "$T"
;;
;;    This prior is important because otherwise instances of and-BIT
;;    not being found as preproof will have a null confidence, that is
;;    because we cannot know with certainty that an and-BIT is *not* a
;;    preproof as it may require to explore the entire univers of
;;    proofs to rule it out. But a null confidence would count as
;;    nothing (according to an extension of the formula in Section
;;    2.4.1.1 of the PLN book to consider uncertainty) which would
;;    make it hard to compare the performances of two predictors since
;;    if they have only positive examples, then the one with the
;;    higher number of examples would win, even if it less accurate,
;;    which isn't right. Indeed the reason is because not finding a
;;    proof does actually bring a bit of information, since there are
;;    so many more non-proofs that proofs, a random and-BIT is likely
;;    not a preproof. That is what this prior captures.
(define and-bit-prior
  (ImplicationScope (stv 0.0001 0.001)
    (VariableList
      (TypedVariable
        (Variable "$A")
        (Type "DontExecLink"))
      (Variable "$T"))
    (And
      (Evaluation
        (Predicate "URE:BC:and-BIT")
        (Variable "$A"))
      (Evaluation
        (Predicate "URE:BC:target")
        (Variable "$T")))
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
(define and-bit-prior-rule
  (car (apply-rule conditional-full-instantiation-implication-scope-meta-rule
                   and-bit-prior)))
(define and-bit-prior-rule-name (DefinedSchemaNode "and-bit-prior-rule"))
(Define and-bit-prior-rule-name and-bit-prior-rule)

;; Define rule base
(define abp-rbs (ConceptNode "and-bit-prior-rule-base"))
(InheritanceLink
   abp-rbs
   (ConceptNode "URE")
)

;; Define abp-bc for convenience
(define (abp-bc . args)
  (apply cog-bc (cons abp-rbs args)))

;; List the rules
(define rules
  (list
     and-bit-prior-rule-name
     fuzzy-conjunction-introduction-2ary-rule-name
  )
)

;; Associate rules to rule base
(ure-add-rules abp-rbs rules)

;; Termination criteria parameters. We need at least 3 iterations, 1
;; to initial the BIT, and 2 to combine and-bit-prior-rule and
;; conjunction, but because some iterations are wasted by checking if
;; some valid rules are left we double this number, thus 6.
(ure-set-num-parameter abp-rbs "URE:maximum-iterations" 6)
