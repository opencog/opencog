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
    (let* ((semantics-list (shuffle impl-links))
        (semantics (select-highest-tv-semantics semantics-list))
        (logic (if (equal? 'ImplicationLink (cog-type semantics))
                   (implication-to-evaluation-s2l (gar semantics)
                                                  (gdr semantics))
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

(define-public (do-pln-QA)
"
  Fetch the semantics with the highest strength*confidence that
  contains words in common with the query
"
    (cog-logger-info "[PLN-Action] do-pln-QA")

    (State pln-qa process-started)
    ; FIXME Why doesn't the first call of (update-inferences) work?
    (update-inferences)
    (update-inferences)
    (let ((inferences (search-inferred-atoms)))
        (if (null? inferences)
            (State pln-answers no-result)
            (choose-response-for-trail-1 (filter-using-query-words
                (cog-outgoing-set inferences) (get-input-utterance-names)))
        )
        (State pln-qa process-finished)
    )
)
