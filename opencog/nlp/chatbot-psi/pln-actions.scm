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

(define (get-sureal-result semantics-list)
    (define semantics (select-highest-tv-semantics semantics-list))

    ; Assuming 'semantics' is an ImplicationLink
    (define logic (implication-to-evaluation-s2l (gar semantics) (gdr semantics)))

    (define sureal-result (sureal logic))

    (if (null? sureal-result)
        ; SuReal may give nothing because sometimes R2L generates the same pattern
        ; (that PLN is looking for) for words that have very different disjuncts.
        ; This introduces 'noise' and confuses SuReal so it generates nothing if
        ; the 'noisy' one got chosen. So try again with other implications in the
        ; semantics-list until the list is empty.
        ; This is kind of like a workaround, would be better to fix in R2L, but
        ; that requires too much work and R2L may be replaced by something better
        ; shortly in the future...
        (if (eq? 1 (length semantics-list))
            '()
            (get-sureal-result (delete semantics semantics-list))
        )
        (first sureal-result)
    )
)

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
           (not-null-iu-inter?
                (lambda (x) (not (null? (iu-inter (list (first (second x))))))))
           (filtered-in (filter not-null-iu-inter? assoc-inferred-names))
           (semantics-list (shuffle (map first filtered-in)))
           (sureal-word-list (get-sureal-result semantics-list))
          )

      (cog-logger-debug "[PLN-Action] assoc-inferred-names = ~a"
                        assoc-inferred-names)
      (cog-logger-debug "[PLN-Action] filtered-in = ~a" filtered-in)
      (cog-logger-debug "[PLN-Action] semantics-list = ~a" semantics-list)
      (cog-logger-debug "[PLN-Action] sureal-word-list = ~a" sureal-word-list)

      (State pln-answers (if (null? sureal-word-list)
                             no-result
                             (List (map Word sureal-word-list))))

      (State pln-qa process-finished)
    )
)
