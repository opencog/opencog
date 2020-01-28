#!

Input = Reptiles can breathe
Input = Lizards are reptiles
|-
output = Lizards can breathe

!#

; Since this file isn't loaded when ghost-procedures is loaded, import required
; modules. It is not loaded with the module it is an experimental feature.
(use-modules (opencog pln)
             (opencog ure))

; NOTE: get-abstract-version could be used for this example but this workflow
; is closer to the generic approach in the long term.
(define r2l-post-processing-1-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$pred")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$pred-inst")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-inst")
        (Type "ConceptNode")))
    (And
      (Evaluation
        (Variable "$pred-inst")
        (List (Variable "$concept-inst")))
      (Implication
        (Variable "$pred-inst")
        (Variable "$pred"))
      (Inheritance
        (Variable "$concept-inst")
        (Variable "$concept")))
    (Evaluation
      (Variable "$pred")
      (List (Variable "$concept"))))
)

(define r2l-post-processing-2-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$pred")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-2")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$pred-inst")
        (Type "PredicateNode"))
      (TypedVariable
        (Variable "$concept-inst-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-inst-2")
        (Type "ConceptNode")))
    (And
      (Evaluation
        (Variable "$pred-inst")
        (List
          (Variable "$concept-inst-1")
          (Variable "$concept-inst-2")))
      (Implication
        (Variable "$pred-inst")
        (Variable "$pred"))
      (Inheritance
        (Variable "$concept-inst-1")
        (Variable "$concept-1"))
      (Inheritance
        (Variable "$concept-inst-2")
        (Variable "$concept-2")))
    (Evaluation
      (Variable "$pred")
      (List
        (Variable "$concept-1")
        (Variable "$concept-2"))))
)

(define r2l-post-processing-3-rule
  (Bind
    (VariableList
      (TypedVariable
        (Variable "$concept-1")
        (Type "ConceptNode"))
      (TypedVariable
        (Variable "$concept-2")
        (Type "ConceptNode")))
    (Evaluation
      (Predicate "are")
      (List
        (Variable "$concept-1")
        (Variable "$concept-2")))
    (Inheritance
      (Variable "$concept-1")
      (Variable "$concept-2")))
)

(define (configure-pln-rbs-3)
    (define rb (ConceptNode "r2l-pln-3"))

    ; The deduction rule doesn't work when the truth-value are extremely
    ; low. (load-gtwc) wasn't helpful
    ;
    ; TODO: use pln-load-rules when move to new PLN API, see
    ; https://github.com/opencog/pln/blob/master/opencog/pln/README.md
    ;
    ;(load-from-path (pln-rule-type->filename "term/deduction"))
    (load-from-path (pln-rule-type->filename "wip/evaluation-to-member"))
    (load-from-path (pln-rule-type->filename "wip/member-to-inheritance"))
    (load-from-path (pln-rule-type->filename "wip/temp-deduction"))
    (load-from-path (pln-rule-type->filename "wip/evaluation-to-member"))
    (load-from-path (pln-rule-type->filename "wip/inheritance-to-member"))
    (load-from-path (pln-rule-type->filename "wip/member-to-evaluation"))

    ; NOTE: The number has no relevance in r2l-mode
    (ure-define-rbs rb 0)
    ; Not sure why
    (ure-set-fuzzy-bool-parameter rb "URE:attention-allocation" 0)
;Not needed because pln-get-nlp-inputs creats required atoms.
;    (ure-define-add-rule rb "r2l-post-processing-1-rule"
;         r2l-post-processing-1-rule (stv 1 1))
;    (ure-define-add-rule rb "r2l-post-processing-2-rule"
;         r2l-post-processing-2-rule (stv 1 1))
    (ure-define-add-rule rb "r2l-post-processing-3-rule"
         r2l-post-processing-3-rule (stv 1 1))
    (ure-define-add-rule rb "evaluation-to-member-1-rule"
         evaluation-to-member-1-rule (stv 1 1))
    (ure-define-add-rule rb "member-to-inheritance-rule"
         member-to-inheritance-rule (stv 1 1))
    ;(ure-define-add-rule rb "deduction-inheritance-rule"
    ;    deduction-inheritance-rule (stv 1 1))
    (ure-define-add-rule rb "temp-deduction-inheritance-rule"
        temp-deduction-inheritance-rule (stv 1 1))
    (ure-define-add-rule rb "inheritance-to-member-rule"
        inheritance-to-member-rule (stv 1 1))
    (ure-define-add-rule rb "member-to-evaluation-1-rule"
         member-to-evaluation-1-rule (stv 1 1))

    ; Return the rule-base
    rb
)

;; Define rulebases
(define rb-trail-3 (configure-pln-rbs-3))

;; Example usage of rb-trail-3
(define (eg-trail-3 steps)
"
  eg-trail-3 STEPS

  Run rb-trail-3 and return outputs that are valid for sureal. 4 steps gives
  the desired result
"
  ; For SuReal
  (nlp-parse "cats can read")

  ; Inputs
  (sleep 1) ; The delay is added to not pollute pln inputs.
  (pln-record-current-time)
  (ghost "Reptiles can breathe")
  (ghost "Lizards are reptiles")

  ; Inference and filtered output.
  (update-inferences rb-trail-3 steps (pln-get-recorded-time))
  (pln->sureal rb-trail-3)
)

; Temporary schemas

(define-public (do_pln)
  (update-inferences rb-trail-3 4 (pln-get-recorded-time))
  (cog-set-value! rb-trail-3 (Node "answer") (Concept (pln->sureal rb-trail-3)))
  (Node "return")
)
(define-public (get_pln_answer)
  (cog-value rb-trail-3 (Node "answer"))
)
