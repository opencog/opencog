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
;; Then the following rule allows to infer that "A" is less or equal
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

;; Warning: don't reload utilities.scm here, otherwise the icl-logger
;; gets its parameters reset.

(define (gen-knowledge-base)
  (let* ((letters (gen-letters))
         (letter-members (map gen-letter-member letters))
         (letter-pairs (map gen-letter-pair (cdr letters) (drop-right letters 1)))
         (X (Variable "$X"))
         (A (Concept "A"))
         (ConceptT (Type "ConceptNode")))
    (ImplicationScope (stv 1 1)
      (TypedVariable X ConceptT)
      (Member X upcase-latin-alphabet)
      (Inheritance A X))))

(define (gen-letters)
  (map gen-letter alphabet-list (iota 26)))

(define (gen-letter X i)
  (Concept (list->string (list X)) (stv (/ (+ i 1) 26) 1)))

(define upcase-latin-alphabet
  (Concept "upcase-latin-alphabet"))

(define (gen-letter-member X)
  (Member (stv 1 1) X upcase-latin-alphabet))

(define (gen-letter-pair X Y)
  (Inheritance (stv 1 1) X Y))

(gen-knowledge-base)
