;; AtomSpace containing the control rules
(define control-as (cog-new-atomspace))

(define (mk-control-rules)
  ;; Copy history-as to the default atomspace
  (clear)
  (cp-as history-as (cog-atomspace))

  ;; Evaluate all antecedents to allow direct evaluation of the
  ;; control rules.
  (icl-logger-debug "Evaluate antecedents")
  (evaluate-antecedents)

  (icl-logger-fine "mk-control-rules (cog-atomspace) [after evaluating antecedents]:")
  (icl-logger-fine-atomspace (cog-atomspace))

  ;; ;; Run pattern miner on the history to find frequent patterns for
  ;; ;; control rules.
  ;; (icl-logger-debug "Mine control rules")
  ;; (mine-control-rules)

  ;; TODO: doesn't work due to conditional-direct-evaluation wrapping quotes around values
  ;; We ground the control rules first.
  (icl-logger-debug "Ground control rules")
  (ground-control-rules)

  (icl-logger-fine "mk-control-rules (cog-atomspace) [after mining control rules]:")
  (icl-logger-fine-atomspace (cog-atomspace))

  ;; Evaluate inference control rules
  (ure-logger-flush)               ; make sure the next message does not get mangled
  (icl-logger-debug "Evaluate control rules")
  (let* ((results (evaluate-control-rules)))
    ;; Copy inference control rules to the Inference Control Rules
    ;; atomspace
    (icl-cp control-as (cog-outgoing-set results)))

  ;; Remove informationless rules from control-as
  (remove-dangling-atoms control-as)

  (icl-logger-debug "Control AtomSpace:")
  (icl-logger-debug-atomspace control-as)
)

(define (ground-control-rules)
  ;; Load PLN to have access to the PLN rules
  (load "pln-rb.scm")

  ;; Ground rules
  ;;
  ;; (preproof-of $A $T)
  ;; (expand ($A $L $R) $B)
  ;; ->
  ;; (preproof-of $B $T)
  (icl-logger-debug "Ground context free control rules")
  (ground-context-free-rules)

  ;; Ground rules like
  ;;
  ;; (preproof-of $A $T)
  ;; (expand ($A (Inheritance a $X) $R) $B)
  ;; ->
  ;; (preproof-of $B $T)
  (icl-logger-debug "Ground a-pattern control rules")
  (ground-a-pattern-rules)
)

(define (ground-context-free-rules)
  (let* ((vardecl (TypedVariable
                    (Variable "$Rule")
                    (Type "DefinedSchemaNode")))
         (impl-vardecl (VariableList
                         (Variable "$T")
                         (dontexec-typed (Variable "$A"))
                         (Variable "$L")
                         (dontexec-typed (Variable "$B"))))
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
                         (dontexec-typed (Variable "$A"))
                         (Variable "$X")
                         (dontexec-typed (Variable "$B"))))
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
                              (DontExec (Variable "$B")))))
         ;; Evaluate all antecedents
         (results (pp-icr-bc impl-antecedent)))

    (icl-logger-fine "evaluate-antecedents = ~a" results)

    ;; Remove query to clean the atomspace
    (extract-hypergraph impl-antecedent) ; TODO: fix bug should print only (stv 1 1)

    ;; Remove SetLink wrapping the results
    (cog-extract results)

    ;; Remove informationless atoms
    (remove-dangling-atoms (cog-atomspace))

    (icl-logger-fine "evaluate-antecedents (cog-atomspace):")
    (icl-logger-fine-atomspace (cog-atomspace))
  )
)

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
(define (evaluate-control-rules)
  ;; Load rule base for evaluating the control rules via direct evaluation
  (load "icr-rb.scm")

  (icl-logger-fine "evaluate-control-rules (cog-atomspace):")
  (icl-logger-fine-atomspace (cog-atomspace))

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

    (icl-logger-fine "evaluate-control-rules control-rules-vardecl = ~a"
                     control-rules-vardecl)
    (icl-logger-fine "evaluate-control-rules control-rules-target = ~a"
                     control-rules-target)

    ;; Produce the inference control rules
    (icr-bc control-rules-target #:vardecl control-rules-vardecl)))
