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
;;   (Concept "Bob")
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

;; (3) Infer the TV of the implicant of (2) using ???
;;
;; Result should be:
;;
;; (Lambda (stv ? ?)
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
;; TODO

;; (4) Infer the TV of the implicand of (2) using ???
;;
;; Result should be:
;;
;; (Lambda (stv ? ?)
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
;; TODO

;; (5) Infer that honest people are more likely to become
;; friends. Apply the inversion rule over (2).
;;
;; Result should be:
;;
;; (Implication (stv ? ?)
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
;; TODO: (cog-bind inversion-rule)

;; (6) Infer that honest human acquaintances tend to become
;; friends. Apply rule ?? on (5)
;;
;; Result should be:
;;
;; (Implication (stv ? ?)
;;    (Lambda
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
;;             (Predicate "is-honest")
;;             (Variable "$X"))
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$Y"))
;;          (Evaluation
;;             (Predicate "acquainted")
;;             (List
;;                (Variable "$X")
;;                (Variable "$Y"))))
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
;; TODO: maybe this requires 2 steps

;; (7) Infer that Bob may become a friend. Apply the
;; implication-full-instantiation-rule on (6).
;;
;; Result should be:
;;
;; (Evalution (stv ? ?)
;;    (Predicate "will-be-friends")
;;    (List
;;       (Concept "Self")
;;       (Concept "Bob")))
(cog-bind implication-full-instantiation-rule)

;; (8) Infer that Bob is funny. Apply the
;; implication-full-instantiation-rule on the implication stating that
;; people telling jokes are funny in the kb.
;;
;; Result should be:
;;
;; (Evaluation (0.8 0.9)
;;   (Predicate "is-funny")
;;   (Concept "Bob"))
;;
;; Actually, no need as it was infered in (1)
;; (cog-bind implication-full-instantiation-rule)

;; (9) Distribute the scope of the amusing funny equivalence. Apply
;; equivalence-scope-distribution-rule.
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

;; (10) Infer that if X is funny, then X is amusing. Apply the
;; equivalence-to-double-implication-rule over the equivalence between
;; is-funny and is-amusing in the kb.
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

;; (11) Infer that Bob is amusing. Apply implication-full-instantiation
;; on the result of (10).
;;
;; Result should be:
;;
;; (Evaluation (stv ? ?)
;;   (Predicate "is-amusing")
;;   (Concept "Bob"))
(cog-bind implication-full-instantiation-rule)

;; (12) Infer that Bob will be an amusing and honest friend. Apply the
;; and-construction-rule over the results of (7), (8) and (11)
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
