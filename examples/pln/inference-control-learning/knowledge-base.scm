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

(define (gen-knowledge-base)
  (gen-knowledge-base-rec (string->list "ABCDEFGHIJKLMNOPQRSTUVWXYZ") 1))

(define (gen-knowledge-base-rec alphabet i)
  (if (< 1 (length alphabet))
      (let* ((A (car alphabet))
             (B (cadr alphabet))
             (tail (cdr alphabet)))
        (gen-letter-order A B i)
        (gen-knowledge-base-rec tail (+ i 1)))))

(define (gen-letter-order A B i)
  (Inheritance (stv 1 1)
    (Concept (list->string (list A)) (stv (/ i 26) 1))
    (Concept (list->string (list B)) (stv (/ (+ i 1) 26) 1))))

(gen-knowledge-base)
