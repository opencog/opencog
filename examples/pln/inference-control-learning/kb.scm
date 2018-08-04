;; The knowledge-base is the upper case latin alphabet order. The order
;; relationship is represented with Inheritance between 2 consecutive
;; letters:
;;
;; Inheritance (stv 1 1)
;;   Concept "a" (stv 1/26 1)
;;   Concept "b" (stv 2/26 1)
;;
;; ...
;;
;; Inheritance (stv 1 1)
;;   Concept "y" (stv 25/26 1)
;;   Concept "z" (stv 26/26 1)
;;
;; The concept strengths, 1/26 to 26/26, are defined to be consistent
;; with PLN semantics, otherwise rules like Inversion by come by
;; inference a lot of crazy shit about the order. Another option would
;; be to use a less-than transitive predicate.
;;
;; One can infer the order between any 2 letters by chaining as many
;; deductions as necessary. On top of that there are shortcuts to
;; infer the order of any pair starting with the letter a, using
;; conditional instantiation.
;;
;; First we need to know that concepts "a" to "z" are letters
;;
;; Member (stv 1 1)
;;   Concept "a"
;;   Concept "latin-alphabet"
;;
;; ...
;;
;; Member (stv 1 1)
;;   Concept "z"
;;   Concept "latin-alphabet"
;;
;; Then either
;;
;; 1. the following rule allows to infer that "a" is less or equal
;; than (inherits from) any other letter
;;
;; ImplicationScope (stv 1 1)
;;   TypedVariable
;;     Variable "$X"
;;     Type "ConceptNode"
;;   Member
;;     Variable "$X"
;;     Concept "latin-alphabet"
;;   Inheritance
;;     Concept "a"
;;     Variable "$X"
;;
;; 2. the following rule
;;
;; ImplicationScope (stv 1 1)
;;   VariableList
;;     TypedVariable
;;       Variable "$X"
;;       Type "ConceptNode"
;;     TypedVariable
;;       Variable "$Y"
;;       Type "ConceptNode"
;;   Evaluation
;;     Predicate "alphabetical-order"
;;     List
;;       Variable "$X"
;;       Variable "$Y"
;;   Inheritance
;;     Variable "$X"
;;     Variable "$Y"
;;
;; combined with the knowledge that an other letter different than a
;; is after a
;;
;; Evaluation (stv 1 1)
;;   Predicate "alphabetical-order"
;;   List
;;     Concept "a"
;;     Concept "b"
;; ...
;; Evaluation (stv 1 1)
;;   Predicate "alphabetical-order"
;;   List
;;     Concept "a"
;;     Concept "z"
;;
;; This is should make the pattern miner free version of icl not yield
;; very good results. The pattern miner version should however yield
;; results comparable to using the smart first rule.
;;
;; Warning: don't reload utilities.scm here, otherwise the icl-logger
;; gets its parameters reset.

(define (gen-knowledge-base)
  (gen-inheritance-contiguous-letters)
  (gen-a-before-all-letters-stupid))

(define (gen-a-before-all-letters-smart)
  (gen-a-inherits-all-letters-smart-rule))

(define (gen-a-before-all-letters-stupid)
  (gen-a-before-all-letters-stupid-facts)
  (gen-X-is-before-Y-then-X-inherits-Y-rule))

(define (gen-inheritance-contiguous-letters)
  (let* ((letters (gen-letters)))
    (map gen-alphabet-letter-member letters)
    (map gen-inheritance-letter-pair (drop-right letters 1) (cdr letters))))

(define (gen-a-inherits-all-letters-smart-rule)
  (let* ((X (Variable "$X"))
         (a (Concept "a"))
         (ConceptT (Type "ConceptNode")))
    (ImplicationScope (stv 1 1)
      (TypedVariable X ConceptT)
      (Member X latin-alphabet)
      (Inheritance a X))))

(define (gen-a-before-all-letters-stupid-facts)
  (let* ((letters (gen-letters))
         (a (Concept "a"))
         (a-before-letters (lambda (X) (gen-alphabetical-order-letter-pair a X))))
    (map a-before-letters (cdr letters))))

(define (gen-X-is-before-Y-then-X-inherits-Y-rule)
  (let* ((X (Variable "$X"))
         (Y (Variable "$Y"))
         (ConceptT (Type "ConceptNode")))
    (ImplicationScope (stv 1 1)
      (VariableList
        (TypedVariable X ConceptT)
        (TypedVariable Y ConceptT))
      (Evaluation alphabetical-order (List X Y))
      (Inheritance X Y))))

(define (gen-letters)
  (map gen-letter alphabet-list (iota 26)))

(define (gen-letter X i)
  (Concept (list->string (list X)) (stv (/ (+ i 1) 26) 1)))

(define latin-alphabet
  (Concept "latin-alphabet"))

(define (gen-alphabet-letter-member X)
  (Member (stv 1 1) X latin-alphabet))

(define (gen-inheritance-letter-pair X Y)
  (Inheritance (stv 1 1) X Y))

(define alphabetical-order
  (Predicate "alphabetical-order"))

(define (gen-alphabetical-order-letter-pair X Y)
  (Evaluation (stv 1 1) alphabetical-order (List X Y)))

(gen-knowledge-base)
