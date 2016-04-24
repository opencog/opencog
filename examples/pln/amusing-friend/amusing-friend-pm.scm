;; PLN inference using solely the pattern matcher for the amusing
;; friend demo

;; Load the background knowledge
(load "kb.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

;; Apply the inference rules using the pattern matcher.

;; We want to infer that Bob will be an amusing and honest friend
;;
;; (And
;;    (Evaluation
;;       (Predicate "will-be-friends")
;;       (List
;;          (Concept "Self")
;;          (Concept "Bob"))
;;    (Evaluation
;;       (Predicate "is-amusing")
;;       (Concept "Bob"))
;;    (Evaluation
;;       (Predicate "is-honest")
;;       (Concept "Bob")

;; (1) Infer that Bob is honest. Apply the
;; implication-full-instantiation-rule on the implication stating that
;; people telling the truth are honest in the kb.
;;
;; Result should be:
;;
;; (Evaluation (0.8 0.9)
;;   (Predicate "is-honest")
;;   (Concept "Bob"))
(cog-bind implication-full-instantiation-rule)

;; (2) Distribute the scope of the implication that friends tend to be
;; honest in the kb, applying implication-scope-distribution-rule.
;;
;; Result should be:
;;
;; (Implication (stv 0.6 0.5)
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y"))))
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$X")))
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$Y"))))
(cog-bind implication-scope-distribution-rule)

;; (3) Infer the TV of the implicant of (2) using
;; lambda-predicate-construction-rule
;;
;; Result should be:
;;
;; (Lambda (stv 1e-5 0.9)
;;    (VariableList
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$Y")
;;          (Type "ConceptNode")))
;;    (Evaluation
;;       (Predicate "will-be-friends")
;;       (List
;;          (Variable "$X")
;;          (Variable "$Y"))))
(cog-bind lambda-predicate-construction-rule)

;; (4) Infer the TV of the implicand of (2) using
;; lambda-predicate-construction-rule
;;
;; Result should be:
;;
;; (Lambda (stv 0.64 0.9)
;;    (VariableList
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$Y")
;;          (Type "ConceptNode")))
;;    (And
;;       (Evaluation
;;          (Predicate "is-honest")
;;          (Variable "$X")))
;;       (Evaluation
;;          (Predicate "is-honest")
;;          (Variable "$Y")))
;; For now inside the kb

;; (5) Infer that honest people are more likely to become
;; friends. Apply the inversion rule over (2).
;;
;; Result should be:
;;
;; (Implication (stv 0.0001171875 0.5)
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$X")))
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$Y"))))
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y"))))
(cog-bind inversion-implication-rule)

;; (6) Distribute the scope of the implication that human
;; acquaintances tend to become friends in the kb, applying
;; implication-scope-distribution-rule.
;;
;; Result should be:
;;
;; (Implication (stv 0.1 0.5)
;;    (LambdaLink (stv 0.001 0.9)
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Inheritance
;;             (Variable "$X")
;;             (Concept "human"))
;;          (Inheritance
;;             (Variable "$Y")
;;             (Concept "human"))
;;          (Evaluation
;;             (Predicate "acquainted")
;;             (List
;;                (Variable "$X")
;;                (Variable "$Y")))))
;;    (LambdaLink (stv 0.0001 0.9)
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y")))))
;;
;; Actually, no need as it was inferred in (2)
;; (cog-bind implication-scope-distribution-rule)

;; (7) Infer that honest human acquaintances tend to become friends
;; (more so than just human acquaintances). Apply rule
;; implication-implicant-conjunction-rule on (6) and the knowledge
;; that human acquantances tend to become friend in the kb.
;;
;; Result should be:
;;
;; (ImplicationLink (stv 0.1171875 0.5)
;;    (AndLink
;;       (LambdaLink (stv 0.63999999 0.89999998)
;;          (VariableList
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (TypedVariableLink
;;                (VariableNode "$Y")
;;                (TypeNode "ConceptNode")
;;             )
;;          )
;;          (AndLink
;;             (EvaluationLink
;;                (PredicateNode "is-honest" (stv 0.80000001 0.89999998))
;;                (VariableNode "$X")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "is-honest" (stv 0.80000001 0.89999998))
;;                (VariableNode "$Y")
;;             )
;;          )
;;       )
;;       (LambdaLink (stv 0.00019999999 0.89999998)
;;          (VariableList
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (TypedVariableLink
;;                (VariableNode "$Y")
;;                (TypeNode "ConceptNode")
;;             )
;;          )
;;          (AndLink
;;             (InheritanceLink
;;                (VariableNode "$X")
;;                (ConceptNode "human")
;;             )
;;             (InheritanceLink
;;                (VariableNode "$Y")
;;                (ConceptNode "human")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "acquainted")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (VariableNode "$Y")
;;                )
;;             )
;;          )
;;       )
;;    )
;;    (LambdaLink (stv 9.9999997e-05 0.89999998)
;;       (VariableList
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (TypedVariableLink
;;             (VariableNode "$Y")
;;             (TypeNode "ConceptNode")
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "will-be-friends" (stv 9.9999997e-05 0.89999998))
;;          (ListLink
;;             (VariableNode "$X")
;;             (VariableNode "$Y")
;;          )
;;       )
;;    )
;; )
(cog-bind implication-implicant-conjunction-rule)

;; (8) TODO: factorize lambda in implicant of (7)

;; (9) Infer that Bob may become a friend. Apply the
;; implication-full-instantiation-rule on (8).
;;
;; Result should be:
;;
;; (Evalution (stv ? ?)
;;    (Predicate "will-be-friends")
;;    (List
;;       (Concept "Self")
;;       (Concept "Bob")))
;;
;; TODO: update the full-instantiation-rule to work on unsugar form
;; (cog-bind implication-full-instantiation-rule)

;; (10) Infer that Bob is funny. Apply the
;; implication-full-instantiation-rule on the implication stating that
;; people telling jokes are funny in the kb.
;;
;; Result should be:
;;
;; (Evaluation (0.8 0.9)
;;   (Predicate "is-funny")
;;   (Concept "Bob"))
;;
;; Actually, no need as it was inferred in (1)
;; (cog-bind implication-full-instantiation-rule)

;; (11) Distribute the scope of the amusing funny equivalence from the
;; kb. Apply equivalence-scope-distribution-rule.
;;
;; Result should be:
;;
;; (Equivalence (stv 0.7 0.9)
;;    (Lambda
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (Evaluation
;;          (Predicate "is-funny")
;;          (Variable "$X")))
;;    (Lambda
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (Evaluation
;;          (Predicate "is-amusing")
;;          (Variable "$X"))))
;; TODO: implement equivalence-scope-distribution-rule

;; (12) Infer that if X is funny, then X is amusing. Apply the
;; equivalence-to-double-implication-rule on (11).
;;
;; Result should be:
;;
;; (Implication (stv 0.7 0.9)
;;    (Lambda
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (Evaluation
;;          (Predicate "is-funny")
;;          (Variable "$X")))
;;    (Lambda
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (Evaluation
;;          (Predicate "is-amusing")
;;          (Variable "$X"))))
(cog-bind equivalence-to-double-implication-rule)

;; (13) Infer that Bob is amusing. Apply implication-full-instantiation
;; on the result of (12).
;;
;; Result should be:
;;
;; (Evaluation (stv ? ?)
;;   (Predicate "is-amusing")
;;   (Concept "Bob"))
(cog-bind implication-full-instantiation-rule)

;; (14) Infer that Bob will be an amusing and honest friend. Apply the
;; and-construction-rule over the results of (9), (10) and (13)
;;
;; Result should be:
;;
;; (And (stv ? ?)
;;    (Evaluation
;;       (Predicate "will-be-friends")
;;       (List
;;          (Concept "Self")
;;          (Concept "Bob"))
;;    (Evaluation
;;       (Predicate "is-amusing")
;;       (Concept "Bob"))
;;    (Evaluation
;;       (Predicate "is-honest")
;;       (Concept "Bob")))
(cog-bind and-construction-rule)
