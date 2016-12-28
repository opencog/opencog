;; PLN actions

(load "pln-utils.scm")

(use-modules (srfi srfi-1))
(use-modules (opencog logger))

;; return atom.tv.s^2*atom.tv.c
(define (higest-tv-fitness atom)
  (let* ((tv (cog-tv atom))
         (tv-s (tv-mean tv))
         (tv-c (tv-conf tv))
         (res (* tv-s tv-s tv-c)))
    ;; (cog-logger-info "higest-tv-fitness(~a) = ~a" atom res)
    res))

;; Select the semantic with highest strength*confidence given a list
;; of semantics
(define (select-highest-tv-semantics semantics-list)
    (max-element-by-key semantics-list higest-tv-fitness))

;; Turn
;;
;; (Implication P Q)
;;
;; into
;;
;; (Word "people")
;; (Set
;;    (Evaluation
;;       Q
;;       (List
;;          (Concept "people")))
;;    (Inheritance
;;       (Concept "people")
;;       (Concept P-name)))))
(define (implication-to-evaluation-s2l P Q)
   (let ((P-name (cog-name P)))
       (Word "people")
       (Set
          (Evaluation
             Q
             (List
                (Concept "people")))
          (Inheritance
             (Concept "people")
             (Concept P-name)))))

(define-public (do-pln-QA)
"
  Fetch the semantics with the highest strength*confidence that
  contains words in common with the query
"
    (cog-logger-debug "[PLN-Action] do-pln-QA")

    (State pln-qa process-started)

    (let* (
           (assoc-inferred-names (get-assoc-inferred-names))
           (iu-names (get-input-utterance-names))
           (iu-inter (lambda (x) (lset-intersection equal? x iu-names)))
           (not-null-iu-inter? (lambda (x) (not (null? (iu-inter (second x))))))
           (filtered-in (filter not-null-iu-inter? assoc-inferred-names))
           (semantics-list (shuffle (map first filtered-in)))
           (semantics (select-highest-tv-semantics semantics-list))
           (semantics-type (cog-type semantics))
           (logic (if (equal? 'ImplicationLink semantics-type)
                      (implication-to-evaluation-s2l (gar semantics)
                                                     (gdr semantics))
                      '()))
           (sureal-result (if (null? logic) '() (sureal logic)))
           (word-list (if (null? sureal-result) '() (first sureal-result)))
          )

      (cog-logger-debug "[PLN-Action] assoc-inferred-names = ~a"
                        assoc-inferred-names)
      (cog-logger-debug "[PLN-Action] filtered-in = ~a" filtered-in)
      (cog-logger-debug "[PLN-Action] semantics-list = ~a" semantics-list)
      (cog-logger-debug "[PLN-Action] semantics = ~a" semantics)
      (cog-logger-debug "[PLN-Action] logic = ~a" logic)
      (cog-logger-debug "[PLN-Action] sureal-result = ~a" sureal-result)
      (cog-logger-debug "[PLN-Action] word-list = ~a" word-list)

      (State pln-answers (if (null? word-list)
                             no-result
                             (List (map Word word-list))))

      (State pln-qa process-finished)
    )
)
