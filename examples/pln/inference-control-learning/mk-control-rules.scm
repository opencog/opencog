;; AtomSpace containing the control rules
(define control-as (cog-new-atomspace))

(define (mk-control-rules)
  ;; Copy history-as to the default atomspace
  (clear)
  (cp-as history-as (cog-atomspace))

  ;; We ground the control rules first then evaluate all antecedents
  ;; to allow direct evaluation of the control rules.
  (ground-control-rules)
  (evaluate-antecedents)

  ;; Produce inference control rules
  (let* ((results (produce-control-rules)))
    ;; Copy inference control rules to the Inference Control Rules
    ;; atomspace
    (icl-cp control-as (cog-outgoing-set results)))

  ;; Remove informationless rules from control-as
  (remove-dangling-atoms control-as)

  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as))

(define (ground-control-rules)
  ;; Load PLN to have access to the PLN rules
  (load "pln-rb.scm")

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
         (impl-antecedent (And
                            (preproof-of
                              (List
                                (Variable "$A")
                                (Variable "$T")))
                            (expand
                              (List
                                (Variable "$A")
                                (Variable "$L")
                                (DontExec (Variable "$Rule")))
                              (Variable "$B"))))
         (impl-consequent (preproof-of
                            (List
                              (Variable "$B")
                              (Variable "$T"))))
         (target (ImplicationScope
                   impl-vardecl
                   impl-antecedent
                   impl-consequent))

         ;; Instantiate the targets as required by icr-bc
         (rules-to-targets (Bind
                             vardecl
                             (Member
                               (Variable "$Rule")
                               pln-rbs)
                             target)))
    (cog-bind rules-to-targets)))

(define (evaluate-antecedents)
  ;; Load rule base for producing evaluating antecedents
  (load "icr-rb.scm")

  (let* ((impl-antecedent (And
                            (preproof-of
                              (List
                                (Variable "$A")
                                (Variable "$T")))
                            (expand
                              (List
                                (Variable "$A")
                                (Variable "$L")
                                (DontExec (Variable "$Rule")))
                              (Variable "$B")))))
    ;; Evaluate all antecedents
    (pp-icr-bc impl-antecedent)))

;; Assumes that the control rules have already been grounded, and all
;; antecedent and consequent groundings are evaluated.
(define (produce-control-rules)
  ;; Load rule base for evaluating the control rules via direct evaluation
  (load "icr-rb.scm")

  (let* ((control-rules-target (Quote
                                 (ImplicationScope
                                   (Unquote
                                     (Variable "$impl-vardecl"))
                                   (And
                                     (preproof-of
                                       (List
                                         (Unquote (Variable "$A"))
                                         (Unquote (Variable "$T"))))
                                     (expand
                                       (List
                                         (Unquote (Variable "$A"))
                                         (Unquote (Variable "$L"))
                                         (Unquote (Variable "$R")))
                                       (Unquote (Variable "$B"))))
                                   (preproof-of
                                     (List
                                       (Unquote (Variable "$B"))
                                       (Unquote (Variable "$T"))))))))

    ;; Produce the inference control rules
    (icr-bc control-rules-target)))
