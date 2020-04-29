;; PLN actions
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog ghost))
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot-psi))
(use-modules (opencog nlp sureal))
(use-modules (opencog logger))
(use-modules (opencog pln))

;; return atom.tv.s^2*atom.tv.c
(define (higest-tv-fitness atom)
  (let* ((tv-s (cog-mean atom))
         (tv-c (cog-confidence atom))
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
(define (get-sureal-result-for-trail-1 semantics-list)
"
  Returns the first surealized ImplicationLink or nil.

  semantics-list: A list of ImplicationLinks
"
    (define semantics (select-highest-tv-semantics semantics-list))

    (define logic
        ; TODO:  Remove this check once other inference-trails are
        ; handled by different response handlers.
        (if (equal? 'ImplicationLink (cog-type semantics))
            (implication-to-evaluation-s2l
                (gar semantics) (gdr semantics))
            '()))

    ; TODO: Remove the check  once other inference-trails are handled
    ; diffferently.
    (define sureal-result (if (nil? logic) '() (sureal logic)))

    (if (nil? sureal-result)
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
            (get-sureal-result-for-trail-1 (delete semantics semantics-list))
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
        (lambda (x) (not (nil?
            (lset-intersection equal? (get-node-names x) query-words))))
        inferences)
)

(define (choose-response-for-trail-1 impl-links)
"
  impl-links: A list of ImplicationLinks
"
    (if (nil? impl-links)
        (State pln-answers no-result)
        (let* ((sureal-word-list (get-sureal-result-for-trail-1 impl-links)))
            (State
                pln-answers
                (if (nil? sureal-word-list)
                    no-result
                    (List (map Word sureal-word-list))
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
    (define time (pln-get-recorded-time))

    (cog-logger-info "[PLN-Action] Started (do-pln-qa)")

    (State pln-qa process-started)
    ; FIXME Why doesn't the first call of (update-inferences) work?
    (update-inferences rb-trail-1 3 time)
    (update-inferences rb-trail-1 3 time)
    (let ((inferences (get-inferred-atoms rb-trail-1)))
        (if (nil? inferences)
            (State pln-answers no-result)
            (choose-response-for-trail-1
                (filter-using-query-words
                    inferences (get-input-utterance-names)))
        )
        (cog-logger-info "[PLN-Action] Finishing (do-pln-qa)")
        (State pln-qa process-finished)
    )

    ; Return for the GroundedSchemaNode
    (Set)
)
