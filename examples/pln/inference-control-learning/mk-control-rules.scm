;; AtomSpace containing the control rules
(define control-as (cog-new-atomspace))

(define (icr-reload)
  (clear)
  (load "icr-rb.scm"))

(define (mk-control-rules)
  ;; Reload the rule base for producing inference control rules
  (icr-reload)

  ;; Copy history-as to the default atomspace
  (cp-as history-as (cog-atomspace))

  ;; Load PLN to have access to the PLN rules
  (load "pln-rb.scm")

  ;; Define BC target and vardecl
  (let* ((vardecl (TypedVariable
                    (Variable "$Rule")
                    (Type "DefinedSchemaNode")))
         (impl-vardecl (VariableList
                         (Variable "$T")
                         (TypedVariable
                           (Variable "$A")
                           (Type "DontExecLink"))
                         (Variable "$L")
                         (TypedVariable
                           (Variable "$B")
                           (Type "DontExecLink"))))
         (impl-antecedant (And
                            (Evaluation
                              (Predicate "URE:BC:preproof-of")
                              (List
                                (Variable "$A")
                                (Variable "$T")))
                            (Execution
                              (Schema "URE:BC:expand-and-BIT")
                              (List
                                (Variable "$A")
                                (Variable "$L")
                                (DontExec (Variable "$Rule")))
                              (Variable "$B"))))
         (impl-consequent (Evaluation
                            (Predicate "URE:BC:preproof-of")
                            (List
                              (Variable "$B")
                              (Variable "$T"))))
         (target (ImplicationScope
                   impl-vardecl
                   impl-antecedant
                   impl-consequent))

         ;; Instantiate the targets as required by icr-bc
         (rules-to-targets (Bind
                             vardecl
                             (Member
                               (Variable "$Rule")
                               pln-rbs)
                             target))
         (targets (cog-bind rules-to-targets))

         ;; Evaluate the antecedants
         (antecedants-result (pp-icr-bc impl-antecedant))

         ;; Produce the inference control rules
         (results (icr-bc target #:vardecl vardecl)))

    ;; Copy inference control rules to the Inference Control Rules
    ;; atomspace, then remove informationless rules.
    (icl-cp control-as (cog-outgoing-set results))
    (remove-dangling-atoms control-as))

  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as))
