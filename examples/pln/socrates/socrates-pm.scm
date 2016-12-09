;; Socrates inference example based on the pattern matcher only. Each
;; step are run manually.

(use-modules (opencog))
(use-modules (opencog atom-types))
(use-modules (opencog exec))
(use-modules (opencog query))
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))

;; Helper to get the r2l output
(define (get-r2l sent-node)
  (interp-get-r2l-outputs (car (sent-get-interp sent-node))))

(define (mock-pln-input sentence)
  (get-r2l (car (nlp-parse sentence))))

; NOTE: Socrates isn't used b/c link-grammar has issue with it
(define input-1 (mock-pln-input "John is a man"))
(define input-2 (mock-pln-input "Men breathe air"))

;; Load PLN rules
(add-to-load-path "../../../opencog/pln/rules")
(load-from-path "deduction-rule.scm")

;; Load PLN meta rules
(add-to-load-path "../../../opencog/pln/meta-rules")
(load-from-path "deduction-rule.scm")

;; Add knowledge to deal with SOG Predicates (Simple Observational
;; Grounded Predicates). Here we are gonna assume that all predicates
;; that have a corresponding WordNode are SOG. Also assumes that the
;; predicate has been built based on the complete cartesian product
;; observations between A and B.
(ImplicationScope
  (VariableList
    (TypedVariable
      (Variable "$W-inst")
      (Type "WordNode"))
    (TypedVariable
      (Variable "$P-inst")
      (Type "PredicateNode"))
    (TypedVariable
      (Variable "$P")
      (Type "PredicateNode"))
    (TypedVariable
      (Variable "$A")
      (Type "ConceptNode"))
    (TypedVariable
      (Variable "$B")
      (Type "ConceptNode"))
    (TypedVariable
      (Variable "$A1")
      (Type "ConceptNode"))
    (TypedVariable
      (Variable "$B1")
      (Type "ConceptNode")))
  (And
    (Implication
      (Variable "$P-inst")
      (Variable "$P"))
    (Reference
      (Variable "$P-inst")
      (Variable "$W-inst"))
    (Evaluation
      (Variable "$P")
      (List
        (Variable "$A")
        (Variable "$B")))
    (Inheritance
      (Variable "$A1")
      (Variable "A"))
    (Inheritance
      (Variable "$B1")
      (Variable "B")))
  (Evaluation
    (Variable "$P")
    (List
      (Variable "$A1")
      (Variable "$B1"))))
