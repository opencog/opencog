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

; Since this file isn't loaded when ghost-procedures is loaded, import required
; modules. It is not loaded with the module it is an experimental feature.
(use-modules (opencog nlp sureal))
(use-modules (opencog ure))

; ----------------------------------------------------------------------------
(define-public (filter-for-pln a-list)
"
  Takes a list of atoms and return a SetLink containing atoms that pln can
  reason on. Some of the patterns that make the returned SetLink are,
          (Inheritance
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")
                  (Type \"SatisfyingSetScopeLink\"))
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")
                  (Type \"SatisfyingSetScopeLink\")))
          (Evaluation
              (Type \"PredicateNode\")
              (ListLink
                  (Type \"ConceptNode\")
                  (Type \"ConceptNode\")))
          (Evaluation
              (Type \"PredicateNode\")
              (ListLink
                  (Type \"ConceptNode\")))
          (Member
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")
                  (Type \"SatisfyingSetScopeLink\"))
              (TypeChoice
                  (Type \"ConceptNode\")
                  (Type \"SatisfyingSetLink\")
                  (Type \"SatisfyingSetScopeLink\")))
          (Implication
              (Type \"PredicateNode\")
              (Type \"PredicateNode\"))
   a-list:
  - This is a list of atoms, for example a list of r2l outputs
"
; TODO: Move this to an (opencog pln) module, when there is one.
    (define filter-in-pattern
        (ScopeLink
            (TypedVariable
                (Variable "$x")
                (TypeChoice
                    (Signature
                        (Inheritance
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink")
                                (Type "SatisfyingSetScopeLink"))
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink")
                                (Type "SatisfyingSetScopeLink"))))
                    (Signature
                        (Implication
                            (Type "PredicateNode")
                            (Type "PredicateNode")))
                    (Signature
                        (Member
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink")
                                (Type "SatisfyingSetScopeLink"))
                            (TypeChoice
                                (Type "ConceptNode")
                                (Type "SatisfyingSetLink")
                                (Type "SatisfyingSetScopeLink"))))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (ListLink
                                (Type "ConceptNode")
                                (Type "ConceptNode"))))
                    (Signature
                        (Evaluation
                            (Type "PredicateNode")
                            (ListLink
                                (Type "ConceptNode"))))
                ))
            ; Return atoms with the given signatures
            (Variable "$x")
        ))
     (define filter-from (SetLink  a-list))
     ; Do the filtering
    (define result (cog-execute! (MapLink filter-in-pattern filter-from)))
     ; Delete the filter-from SetLink and its encompasing MapLink.
    (cog-extract-recursive! filter-from)
     result
)

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
        (if (nil? inputs)
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
  (define (get-sureal-results pln-output)
    (sureal (Set (car (filter
      (lambda (x) (equal? 'EvaluationLink (cog-type x))) pln-output)))))
  (define pln-outputs (cog-value->list (get-inferred-atoms trail)))
  (define candidates
    (if (nil? pln-outputs)
      '()
      (cog-outgoing-set (filter-for-sureal pln-outputs))))

  ; TODO: Add measure to choose between candidates based on query or some
  ; other method.
  (if (nil? candidates)
    ""
    ; FIXME: This only works for trail-3
    (let ((sureal-results (get-sureal-results candidates)))
      (if (nil? sureal-results)
        ""
        (string-join (car (sureal-results))))
    ))
)
