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
      (expand
        (List
          (Variable "$A")
          (Variable "$L")
          (Variable "$R"))
        (Variable "$B"))
      (preproof-of
        (List
          (Variable "$B")
          (Variable "$T"))))
    (preproof-of
      (List
        (Variable "$A")
        (Variable "$T")))))

;; Load useful PLN rules
(add-to-load-path "../../../opencog/pln/")
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

;; Termination criteria parameters. Let n = piter-1, which is the
;; maximum inference size we may obtain from solving a problem, -1
;; comes from the fact the first iteration merely initialize the BIT.
;;
;; We have to grow a chain of preproof expansion of maximum size 2*n
;; because 2 rules must be applied in series, conditional
;; instantiation followed by conjunction introduction. At each
;; iteration, assuming the chain has size i, there are at most i/2+1
;; premises to choose from, i/2 dead-ends and 1 correct, to understand
;; why you may look at the inference tree example
;;
;;  [12683283867560939830][14] [12823859965981897975][14]
;;  =======fuzzy-conjunction-introduction-formula========
;;               [16933538436290672077][14] [10726136630260592600][3]
;;               ----conditional-full-instantiation-scope-formula----
;; [16373111896861386537][14] [17721967112989671184][14]
;; =======fuzzy-conjunction-introduction-formula========
;;              [14468762034246325305][14] [10726136630260592600][3]
;;              ----conditional-full-instantiation-scope-formula----
;;                           [10889719491974701818][3]
;;
;; here i=4, the correct premise to expand is either
;; [12683283867560939830][14] or [12823859965981897975][14], while
;; [16373111896861386537][14] is surely a dead-end but may still be
;; choosen for expansion during this iteration. Premise
;; [10726136630260592600][3] won't be expanded because it has a
;; confidence of 1. So the number of premises that can be expanded is
;; 3, or 4/2+1.
;;
;; Thus in the course of the inference the maximum number of
;; iterations we may need is
;;
;; 1 + sum_i=0^(2*n) i/2+1
;;
;; 1 for initializing the BIT, and the rest for the sum of all
;; possible expansions assuming the inference has maximum size i so
;; far.
;;
;; Which can be simplified into
;;
;; (piter + 1) / 2 + piter^2
;;
;; TODO: it's still expensive. I can think of 2 other options 1. write
;;       or even better learn control rules to speed that up.
;;       2. iterate piter-1 times instantiation . conjunction
(ure-set-num-parameter pep-rbs "URE:maximum-iterations"
                       (ceiling (exact->inexact (+ (/ (+ piter 1) 2)
                                                   (* piter piter)))))

;; Complexity penalty, negative so that it always expands the biggest,
;; we can do that since the backward chainer is linear (it has only
;; one branch).
(ure-set-num-parameter pep-rbs "URE:complexity-penalty" -2)

;; Actually: to speed this up we instead reapply the following rule,
;; corresponding to a conditional instantiation followed by a
;; conjunction, piter-1 times
(define pep-rule
(BindLink
  (VariableList
    (TypedVariableLink
      (VariableNode "$A")
      (TypeNode "DontExecLink")
    )
    (VariableNode "$T")
    (TypedVariableLink
      (VariableNode "$B-cc00185")
      (TypeNode "DontExecLink")
    )
    (TypedVariableLink
      (VariableNode "$R-7b43bfa2")
      (TypeNode "DontExecLink")
    )
    (VariableNode "$L-359df4eb")
  )
  (AndLink
    (preproof-of
      (ListLink
        (VariableNode "$B-cc00185")
        (VariableNode "$T")
      )
    )
    (expand
      (ListLink
        (VariableNode "$A")
        (VariableNode "$L-359df4eb")
        (VariableNode "$R-7b43bfa2")
      )
      (VariableNode "$B-cc00185")
    )
  )
  (ExecutionOutputLink
    (GroundedSchemaNode "scm: conditional-full-instantiation-scope-formula")
    (ListLink
      (preproof-of
        (ListLink
          (VariableNode "$A")
          (VariableNode "$T")
        )
      )
      (ExecutionOutputLink
        (GroundedSchemaNode "scm: fuzzy-conjunction-introduction-formula")
        (ListLink
          (AndLink
            (preproof-of
              (ListLink
                (VariableNode "$B-cc00185")
                (VariableNode "$T")
              )
            )
            (expand
              (ListLink
                (VariableNode "$A")
                (VariableNode "$L-359df4eb")
                (VariableNode "$R-7b43bfa2")
              )
              (VariableNode "$B-cc00185")
            )
          )
          (SetLink
            (preproof-of
              (ListLink
                (VariableNode "$B-cc00185")
                (VariableNode "$T")
              )
            )
            (expand
              (ListLink
                (VariableNode "$A")
                (VariableNode "$L-359df4eb")
                (VariableNode "$R-7b43bfa2")
              )
              (VariableNode "$B-cc00185")
            )
          )
        )
      )
      (ImplicationScopeLink (stv 1.000000 1.000000)
        (VariableList
          (TypedVariableLink
            (VariableNode "$A")
            (TypeNode "DontExecLink")
          )
          (TypedVariableLink
            (VariableNode "$B")
            (TypeNode "DontExecLink")
          )
          (TypedVariableLink
            (VariableNode "$R")
            (TypeNode "DontExecLink")
          )
          (VariableNode "$L")
          (VariableNode "$T")
        )
        (AndLink
          (expand
            (ListLink
              (VariableNode "$A")
              (VariableNode "$L")
              (VariableNode "$R")
            )
            (VariableNode "$B")
          )
          (preproof-of
            (ListLink
              (VariableNode "$B")
              (VariableNode "$T")
            )
          )
        )
        (preproof-of
          (ListLink
            (VariableNode "$A")
            (VariableNode "$T")
          )
        )
      )
    )
  )
)
)
