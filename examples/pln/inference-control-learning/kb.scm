;; The knowledge-base is the upper case latin alphabet order. The order
;; relationship is represented with Inheritance between 2 consecutive
;; letters:
;;
;; Inheritance (stv 1 1)
;;   Concept "A" (stv 1/26 1)
;;   Concept "B" (stv 2/26 1)
;;
;; ...
;;
;; Inheritance (stv 1 1)
;;   Concept "Y" (stv 25/26 1)
;;   Concept "Z" (stv 26/26 1)
;;
;; The concept strengths, 1/26 to 26/26, are defined to be consistent
;; with PLN semantics, otherwise rules like Inversion by come by
;; inference a lot of crazy shit about the order. Another option would
;; be to use a less-than transitive predicate.
;;
;; One can infer the order between any 2 letters by chaining as many
;; deductions as necessary. On top of that there are shortcuts to
;; infer the order of any pair starting with the letter A, using
;; conditional instantiation.
;;
;; First we need to know that concepts "A" to "Z" are letters
;;
;; Member (stv 1 1)
;;   Concept "A"
;;   Concept "latin-alphabet"
;;
;; ...
;;
;; Member (stv 1 1)
;;   Concept "Z"
;;   Concept "upcase-latin-alphabet"
;;
;; Then either
;;
;; 1. the following rule allows to infer that "A" is less or equal
;; than (inherits from) any other letter
;;
;; ImplicationScope (stv 1 1)
;;   TypedVariable
;;     Variable "$X"
;;     Type "ConceptNode"
;;   Member
;;     Variable "$X"
;;     Concept "upcase-latin-alphabet"
;;   Inheritance
;;     Concept "A"
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
;; combined with the knowledge that an other letter different than A
;; is after A
;;
;; Evaluation
;;   Predicate "alphabetical-order"
;;   List
;;     Concept "A"
;;     Concept "B"
;; ...
;; Evaluation
;;   Predicate "alphabetical-order"
;;   List
;;     Concept "A"
;;     Concept "Z"
;;
;; This is should make the pattern miner free version of icl not yield
;; very good results. The pattern miner version should however yield
;; results comparable to using the smart first rule.
;;
;; Warning: don't reload utilities.scm here, otherwise the icl-logger
;; gets its parameters reset.

(define (gen-knowledge-base)
  (gen-inheritance-contiguous-letters)
  (gen-A-before-all-letters-smart))

(define (gen-A-before-all-letters-smart)
  (gen-A-inherits-all-letters-smart-rule))

(define (gen-A-before-all-letters-stupid)
  (gen-A-before-all-letters-stupid-facts)
  (gen-X-is-before-Y-then-X-inherits-Y-rule))

(define (gen-inheritance-contiguous-letters)
  (let* ((letters (gen-letters)))
    (map gen-alphabet-letter-member letters)
    (map gen-inheritance-letter-pair (drop-right letters 1) (cdr letters))))

(define (gen-A-inherits-all-letters-smart-rule)
  (let* ((X (Variable "$X"))
         (A (Concept "A"))
         (ConceptT (Type "ConceptNode")))
    (ImplicationScope (stv 1 1)
      (TypedVariable X ConceptT)
      (Member X upcase-latin-alphabet)
      (Inheritance A X))))

(define (gen-A-before-all-letters-stupid-facts)
  (let* ((letters (gen-letters))
         (A (Concept "A"))
         (A-before-letters (lambda (X) (gen-alphabetical-order-letter-pair A X))))
    (map A-before-letters (cdr letters))))

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

(define upcase-latin-alphabet
  (Concept "upcase-latin-alphabet"))

(define (gen-alphabet-letter-member X)
  (Member (stv 1 1) X upcase-latin-alphabet))

(define (gen-inheritance-letter-pair X Y)
  (Inheritance (stv 1 1) X Y))

(define alphabetical-order
  (Predicate "alphabetical-order"))

(define (gen-alphabetical-order-letter-pair X Y)
  (Evaluation (stv 1 1) alphabetical-order (List X Y)))

(gen-knowledge-base)
