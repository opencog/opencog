;; Background PLN reasoning
;;
;; Very simplistic and hacky at the moment, loop of 2 rules
;;
;; 1. Preprocessing to turn r2l outputs into something that PLN can reason on
;;
;; 2. Limited induction reasoning
;;
;; The reasoner doesn't use the URE. Instead if merely applies the 2
;; rules one after the other in a loop.
;;
;; You may test it as following (wait for a couple seconds between
;; each command to be sure that the chatbot-psi and the pln-reasoner
;; have time to diggest them.
;;
;; guile -l main.scm
;;
;; ;; positive sentence, also helps sureal to generate answer
;; (mock-HEAD-chat "p-1" "Eddie" "small animals are cute")
;;
;; ;; positive sentences
;; (mock-HEAD-chat "p-2" "Ruiting" "birds are lovely")
;; (mock-HEAD-chat "p-3" "Ben" "dogs are awesome")
;; (mock-HEAD-chat "p-3" "Ben" "the multiverse is beautiful")
;;
;; ;; negative sentences
;; (mock-HEAD-chat "p-3" "Ben" "I hate to relax, it makes me nervous")
;;
;; ;; Statement about Ben
;; (mock-HEAD-chat "p-1" "Eddie" "Ben is crazy")
;;
;; ;; Question about happiness. Answer should be: crazy people are happy
;; (mock-HEAD-chat "p-1" "Eddie" "What do you know about happy?")

(use-modules (opencog))
(use-modules (opencog atom-types))
(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (opencog rule-engine))
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))
(use-modules (srfi srfi-1))

(load "pln-utils.scm")
(load "pln-trail-1.scm")
(load "pln-trail-2.scm")

;-------------------------------------------------------------------------------
(define (infer-on-r2l rule-base r2l-outputs steps)
    (let* ((inference-results
                (simple-forward-chain rule-base r2l-outputs steps))
          (clean-results (lset-difference equal? inference-results r2l-outputs))
          (results-for-sureal
                (cog-outgoing-set (filter-for-sureal clean-results)))
          )
        results-for-sureal
    )
)

;-------------------------------------------------------------------------------
;;;;;;;;;;
;; Main ;;
;;;;;;;;;;
; FIXME: The reocrding starts on loading. Should only recored inputs used
; for inference.
(pln-record-current-time)

(define pln-update-count 0)
(define-public (pln-get-update-count) pln-update-count)

(define (update-inferences)
    (cog-logger-info "[PLN-Action] Started (update-inferences)")

    ;; Apply Implication direct evaluation (and put the result in
    ;; pln-inferred-atoms state)
    (let* ((inputs (pln-get-nlp-inputs (get-previous-said-sents
                (pln-get-recorded-time))))
        (inferences (infer-on-r2l rb-trail-1 inputs 3)))

        (if (not (null? inferences))
            (add-to-pln-inferred-atoms inferences))
    )

    (set! pln-update-count (+ pln-update-count 1))
    (cog-logger-info "[PLN-Action] Finished (update-inferences)")
)
