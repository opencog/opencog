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

  ;; Ground rules
  ;;
  ;; (preproof-of $A $T)
  ;; (expand ($A $L $R) $B)
  ;; ->
  ;; (preproof-of $B $T)
  (ground-context-free-rules)

  ;; Ground rules like
  ;;
  ;; (preproof-of $A $T)
  ;; (expand ($A (Inheritance a $X) $R) $B)
  ;; ->
  ;; (preproof-of $B $T)
  (ground-a-pattern-rules)
)

(define (ground-context-free-rules)
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
                             target))
         (results (cog-execute! rules-to-targets)))

    ;; Remove query to avoid then confounding it with a control rule
    (extract-hypergraph rules-to-targets)

    results))

(define (ground-a-pattern-rules)
  (let* ((vardecl (TypedVariable
                    (Variable "$Rule")
                    (Type "DefinedSchemaNode")))
         (impl-vardecl (VariableList
                         (Variable "$T")
                         (TypedVariable
                           (Variable "$A")
                           (Type "DontExecLink"))
                         (Variable "$X")
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
                                (Inheritance
                                  (Concept "a")
                                  (Variable "$X"))
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
                             target))
         (results (cog-execute! rules-to-targets)))

    ;; Remove query to avoid then confounding it with a control rule
    (extract-hypergraph rules-to-targets)

    results))

(define (evaluate-antecedents)
  ;; Load rule base for producing evaluating antecedents
  (load "icr-rb.scm")

  (let* ((impl-antecedent (And
                            (preproof-of
                              (List
                                (DontExec (Variable "$A"))
                                (Variable "$T")))
                            (expand
                              (List
                                (DontExec (Variable "$A"))
                                (Variable "$L")
                                (DontExec (Variable "$Rule")))
                              (DontExec (Variable "$B"))))))
    ;; Evaluate all antecedents
    (pp-icr-bc impl-antecedent)))

;; Assumes that the control rules have already been grounded, and all
;; antecedent and consequent groundings are evaluated, merely run the
;; backward chainer on the following query
;;
;; Quote
;;   ImplicationScope
;;     Unquote "$impl-vardecl"
;;     And
;;       Evaluation
;;         Predicate "URE:BC:preproof-of"
;;         Unquote Variable "preproof-A-args"
;;       Execution
;;         GroundedSchema "URE:BC:expand-and-BIT"
;;         Unquote Variable "$expand-inputs"
;;         Unquote Variable "$expand-output"
;;     Evaluation
;;       Predicate "URE:BC:preproof-of"
;;       Unquote Variable "$preproof-B-args"
;;
;; with the following variable declaration for the query
;;
;; VariableList
;;   TypedVariable
;;     Variable "$impl-vardecl"
;;     (Type "VariableList")
;;   TypedVariable
;;     Variable "$preproof-A-args"
;;     Type "ListLink"
;;   TypedVariable
;;     Variable "$expand-inputs"
;;     Type "ListLink"
;;   Variable "$expand-output"
;;   TypedVariable
;;     Variable "$preproof-B-args"
;;     Type "ListLink"
;;
;; TODO: Probably make the arguments as explicit as possible, to
;; express the relationships between the different parts, as to make
;; sure the variable declarations will reflect these.
(define (produce-control-rules)
  ;; Load rule base for evaluating the control rules via direct evaluation
  (load "icr-rb.scm")

  (let* ((control-rules-vardecl (VariableList
                                  (TypedVariable
                                    (Variable "$impl-vardecl")
                                    (Type "VariableList"))
                                  (TypedVariable
                                    (Variable "$preproof-A-args")
                                    (Type "ListLink"))
                                  (TypedVariable
                                    (Variable "$expand-inputs")
                                    (Type "ListLink"))
                                  (Variable "$expand-output")
                                  (TypedVariable
                                    (Variable "$preproof-B-args")
                                    (Type "ListLink"))))
         (control-rules-target (Quote
                                 (ImplicationScope
                                   (Unquote
                                     (Variable "$impl-vardecl"))
                                   (And
                                     (preproof-of
                                       (Unquote (Variable "$preproof-A-args")))
                                     (expand
                                       (Unquote (Variable "$expand-inputs"))
                                       (Unquote (Variable "$expand-output"))))
                                   (preproof-of
                                     (Unquote (Variable "$preproof-B-args")))))))

    ;; Produce the inference control rules
    (icr-bc control-rules-target #:vardecl control-rules-vardecl)))
