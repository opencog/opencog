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
(load "pln-trail-3.scm")

;-------------------------------------------------------------------------------
(define (infer-on-r2l rule-base r2l-outputs steps)
    (let* ((inference-results
                (simple-forward-chain rule-base r2l-outputs steps))
          (clean-results
                (lset-difference equal? inference-results r2l-outputs)))
    ;      (filter-for-sureal clean-results)
        clean-results
    )
)

;-------------------------------------------------------------------------------
(define (update-inferences trail steps time)
"
  update-inferences TRAIL STEPS TIME

  Run the simple forward chainer using the TRAIL URE-rulebase over the
  r2l outputs of sentences inputed since TIME(in seconds). The output of the
  inferences are recorded as a value on TRAIL.
"
    (let* ((inputs (pln-get-nlp-inputs (get-previous-said-sents time))))
        (add-to-pln-inferred-atoms trail (infer-on-r2l trail inputs steps))
    )
)
