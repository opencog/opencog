;; PLN actions
(use-modules (srfi srfi-1))
(use-modules (opencog logger))

(load "pln-utils.scm")
(load "pln-reasoner.scm")


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
; TODO: Replace by microplanner.
    (let ((P-name (cog-name P)))
       (Set
          (Evaluation
             Q
             (List
                 ; NOTE: The concept is driven from must-have-names.
                (Concept "people")))
          (Inheritance
             (Concept "people")
             (Concept P-name)))
    )
)

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

(define (get-node-names x)
    (get-names (cog-get-all-nodes x))
)

(define (filter-using-query-words inferences query-words)
"
  Returns a list containing inferences that might be possible response to
  the query or empty list if there aren't any.

  query-words: A list of lower-case words that make the sentence that signals
    query, and that specify about what we want to know about.
  inferences: a list with inference results that are accumulated by
    chaining on the sentences that signaled the start of input.
"

    (filter
        (lambda (x) (not (null?
            (lset-intersection equal? (get-node-names x) query-words))))
        inferences)
)

(define (choose-response-for-trail-1 impl-links)
"
  impl-links: A list of ImplicationLinks
"
    (if (null? impl-links)
        (State pln-answers no-result)
        (let* ((semantics (select-highest-tv-semantics (shuffle impl-links)))
            (logic
                (if (equal? 'ImplicationLink (cog-type semantics))
                    (implication-to-evaluation-s2l
                        (gar semantics) (gdr semantics))
                    '()))
            (sureal-result (if (null? logic) '() (sureal logic)))
            )

            (State
                pln-answers
                (if (null? sureal-result)
                    no-result
                    (List (map Word (first sureal-result)))
                )
            )
        )
    )
)

(define-public (do-pln-qa)
"
  Fetch the semantics with the highest strength*confidence that
  contains words in common with the query
"
    (cog-logger-info "[PLN-Action] Started (do-pln-qa)")

    (State pln-qa process-started)
    ; FIXME Why doesn't the first call of (update-inferences) work?
    (update-inferences)
    (update-inferences)
    (let ((inferences (get-inferred-atoms)))
        (if (null? inferences)
            (State pln-answers no-result)
            (choose-response-for-trail-1
                (filter-using-query-words
                    inferences (get-input-utterance-names)))
        )
        (cog-logger-info "[PLN-Action] Finishing (do-pln-qa)")
        (State pln-qa process-finished)
    )
)
