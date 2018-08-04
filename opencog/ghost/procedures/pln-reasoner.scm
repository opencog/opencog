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
(define inference-inputs-key (Predicate "inference-inputs"))
(define (update-inferences trail steps time)
"
  update-inferences TRAIL STEPS TIME

  Run the simple forward chainer using the TRAIL URE-rulebase over the
  r2l outputs of sentences inputed since TIME(in seconds). The output of the
  inferences are recorded as a value on TRAIL.
"
    (let* ((inputs (pln-get-nlp-inputs (get-previous-said-sents time))))
        (if (null? inputs)
          (error "Inference in requested without proper configuration")
        ; Why record the inputs? -> To be able to filter outputs based on
        ; similarities to the inputs.
          (begin
            (cog-set-value! trail inference-inputs-key inputs)
            (add-to-pln-inferred-atoms trail (infer-on-r2l trail inputs steps))
          )
        )
    )
)

;-------------------------------------------------------------------------------
(define (get-inference-inputs trail)
"
  get-inference-inputs TRAIL

  Retuns a LinkValue of the latest inputs passed to be inferred by using TRAIL
"
  (cog-value trail inference-inputs-key)
)

;-------------------------------------------------------------------------------
(define (pln->sureal trail)
"
  pln->sureal TRAIL

  Surealize and return the string of the TRAIL's inferred results.
"
  (define pln-outputs (cog-value->list (get-inferred-atoms trail)))
  (define candidates
    (if (null? pln-outputs)
      '()
      (cog-outgoing-set (filter-for-sureal pln-outputs))))

  ; TODO: Add measure to choose between candidates based on query or some
  ; other method.
  (if (null? candidates)
    ""
    ; FIXME: This only works for trail-3
    (string-join (car (sureal (Set (car (filter
			(lambda (x) (equal? 'EvaluationLink (cog-type x))) candidates))))))
  )
)
