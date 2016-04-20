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

;; (1) Infer that Bob may become a friend. Apply the
;; implication-full-instantiation-rule on the acquaintance to friend
;; ship implication in the kb.
;;
;; Result should be:
;;
;; (Evalution (stv 0.1 0.5)
;;    (Predicate "will-be-friends")
;;    (List
;;       (Concept "Self")
;;       (Concept "Bob"))
(cog-bind implication-full-instantiation-rule)

;; (2) Infer that Bob is honnest. Apply the
;; implication-full-instantiation-rule on the implication stating that
;; people telling the truth are honest in the kb.
;;
;; Result should be:
;;
;; (Evaluation (0.8 0.9)
;;   (Predicate "is-honest")
;;   (Concept "Bob")
;;
;; Actually, no need as it was infered in (1)
;; (cog-bind implication-full-instantiation-rule)

;; (3) Infer that Bob is funny. Apply the
;; implication-full-instantiation-rule on the implication stating that
;; people telling jokes are funny in the kb.
;;
;; Result should be:
;;
;; (Evaluation (0.8 0.9)
;;   (Predicate "is-funny")
;;   (Concept "Bob")
;;
;; Actually, no need as it was infered in (1)
;; (cog-bind implication-full-instantiation-rule)

;; (4) Infer that if X is funny, then X is amusing. Apply the
;; equivalence-to-double-implication-rule over the equivalence between
;; is-funny and is-amusing in the kb.
;;
;; Result should be:
;;
;; (Implication (stv ? ?)
;;    (TypedVariable
;;       (Variable "$X")
;;       (Type "ConceptNode"))
;;    (Evaluation
;;       (Predicate "is-funny")
;;       (Variable "$X"))
;;    (Evaluation
;;       (Predicate "is-amusing")
;;       (Variable "$X")))
(cog-bind equivalence-to-double-implication-rule)

;; (5) Infer that Bob is amusing. Apply implication-full-instantiation
;; on the result of (4).
;;
;; Result should be:
;;
;; (Evaluation (stv ? ?)
;;   (Predicate "is-amusing")
;;   (Concept "Bob")
(cog-bind implication-full-instantiation-rule)

;; (6) Infer that Bob will be an amusing and honest friend. Apply the
;; and-construction-rule over the results of (1), (2) and (5)
;;
;; Result should be:
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
(cog-bind and-construction-rule)
