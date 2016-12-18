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
; TODO: Remove this loop by integrating the pln-demo to openpsi
(define enable-pln-loop #f)
(define-public (pln-running?) enable-pln-loop)

(define pln-loop-count 0)
(define-public (pln-get-loop-count) pln-loop-count)

(define (pln-loop)
    ;; Apply l2s rules
    (let (;(name-on-last-sentence (put-name-on-the-last-sentence))
        ;(sentiment-sentence-to-person-l2s-results
        ;    (cog-fc (SetLink) rb1 (SetLink)))
        (unary-predicate-speech-act-l2s-results
            (cog-fc (SetLink) rb2 (SetLink)))
        )

        ;(cog-logger-info "[PLN-Reasoner] name-on-last-sentence = ~a"
        ;    name-on-last-sentence)
        ;(cog-logger-info
        ;    "[PLN-Reasoner] sentiment-sentence-to-person-l2s-results = ~a"
        ;    sentiment-sentence-to-person-l2s-results)
        (cog-logger-info
            "[PLN-Reasoner] unary-predicate-speech-act-l2s-results = ~a"
            unary-predicate-speech-act-l2s-results)
    )

    ;; Apply Implication direct evaluation (and put the result in
    ;; pln-inferred-atoms state)
    (let* ((direct-eval-results (cog-fc (SetLink) rb3 (SetLink)))
        ;; Filter only inferred result containing "happy". This is a
        ;; temporary hack to make it up for the lack of attentional
        ;; allocation
        (must-contain (list (Predicate "happy")))
        (ff (lambda (x) (lset<= equal? must-contain (cog-get-all-nodes x))))
        (filtered-results (filter ff (cog-outgoing-set direct-eval-results))))

        (add-to-pln-inferred-atoms (Set filtered-results))
    )

    (cog-logger-info "[PLN-Reasoner] pln-inferred-atoms = ~a"
        (search-inferred-atoms))

    ;; sleep a bit, to not overload the CPU too much
    (cog-logger-info "[PLN-Reasoner] Sleep for a second")
    (set! pln-loop-count (+ pln-loop-count 1))
    (sleep 1)

    ;; Loop
    ;(if enable-pln-loop (pln-loop))
)

(define-public (pln-run)
    (if (not (pln-running?))
        (begin
            (set! enable-pln-loop #t)
            (begin-thread (pln-loop))))
)

(define-public (pln-halt) (set! enable-pln-loop #f))

; Start pln loop
;(pln-run)
