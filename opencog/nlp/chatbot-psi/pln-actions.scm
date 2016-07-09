;; PLN actions

(load "pln-states.scm")
(load "pln-utils.scm")

(use-modules (srfi srfi-1))

;; TODO: call a fuzzy matcher (perhaps the default fuzzy matcher) on the
;; set of PLN inferred knowledge and pick up the one that matches the
;; most with the query
(define-public (do-pln-QA)
    (State pln-qa search-started)

    ;; (cog-logger-info "[PLN-Psi] do-pln-QA")

    (let* (
           (assoc-inferred-names (get-assoc-inferred-names))
           (iu-names (get-input-utterance-names))
           (iu-inter (lambda (x) (lset-intersection equal? x iu-names)))
           (not-null-iu-inter? (lambda (x) (not (null? (iu-inter (second x))))))
           (filtered-in (filter not-null-iu-inter? assoc-inferred-names))
           ;; TODO replace that by random selection
           (semantics (first (first filtered-in)))
           (semantics-type (cog-type semantics))
           (logic (if (equal? 'ImplicationLink semantics-type)
                      (implication-to-evaluation-s2l (gar semantics)
                                                     (gdr semantics))
                      '()))
           (word-list (if (null? logic) '() (sureal logic)))
           )

      ;; (cog-logger-info "[PLN-Psi] assoc-inferred-names = ~a"
      ;;                  assoc-inferred-names)
      ;; (cog-logger-info "[PLN-Psi] iu-names = ~a" iu-names)
      ;; (cog-logger-info "[PLN-Psi] iu-inter = ~a" iu-inter)
      ;; (cog-logger-info "[PLN-Psi] filtered-in = ~a" filtered-in)
      ;; (cog-logger-info "[PLN-Psi] semantics = ~a" semantics)
      ;; (cog-logger-info "[PLN-Psi] logic = ~a" logic)
      ;; (cog-logger-info "[PLN-Psi] word-list = ~a" word-list)

      (State pln-answers (List (map Word (first word-list))))

      ;; (State pln-answers (List (Word "I") (Word "am") (Word "the") (Word "PLN") (Word "answer")))

      (State pln-qa search-finished)
    )
)

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

