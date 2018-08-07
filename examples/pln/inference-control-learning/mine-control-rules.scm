(define (mine-control-rules)
  ;; Filter history, for now remove all root atoms with TV below
  ;; 0.1. That is because the pattern miner doesn't support values
  ;; yet.
  (remove-almost-false-atoms)

  ;; TODO: should do the same for almost true atoms, as to find
  ;; patterns predicting failure.

  ;; Fetch relevant texts and apply the pattern miner to it
  (icl-logger-info "Start mining patterns for control rules")
  (let* ((texts-cpt (mk-texts))
         (mined-patterns
          (cog-mine texts-cpt 3 #:maxiter 100 #:initpat (initpat))))
    (icl-logger-fine "Mined patterns = ~a" mined-patterns)))

;; Create a texts concept and add as members to it all atoms of the form
;;
;; preproof-of(A, T)
;; expand(A, L, R, B)
(define (mk-texts)
  (let* (;; Create texts concept node
         (texts-cpt (random-node 'ConceptNode 8 "texts-"))

         ;; Fetch preproof-of
         (preproof-vardecl (VariableList
                              (TypedVariable (Variable "$I") (Type "DontExecLink"))
                              (Variable "$T")))
         (preproof-pattern (preproof-of (List (Variable "$I") (Variable "$T"))))
         (preproof-bind (Bind preproof-vardecl preproof-pattern preproof-pattern))
         (preproof-results (cog-execute! preproof-bind))

         ;; Fetch expand
         (expand-vardecl (VariableList
                           (TypedVariable (Variable "$A") (Type "DontExecLink"))
                           (TypedVariable (Variable "$B") (Type "DontExecLink"))
                           (TypedVariable (Variable "$R") (Type "DontExecLink"))
                           (Variable "$L")))
         (expand-input (List (Variable "$A") (Variable "$L") (Variable "$R")))
         (expand-pattern (expand expand-input (Variable "$B")))
         (expand-bind (Bind expand-vardecl expand-pattern expand-pattern))
         (expand-results (cog-execute! expand-bind))

         ;; Define function to add a member to texts-cpt
         (add-text (lambda (x) (Member x texts-cpt))))

    ;; Add all fetched preproof-of as members of texts-cpt
    (for-each add-text (cog-outgoing-set preproof-results))

    ;; Add all fetched expand as members of texts-cpt
    (for-each add-text (cog-outgoing-set expand-results))

    (icl-logger-fine "mk-texts members of texts-cpt = ~a" (get-members texts-cpt))

    ;; Return texts concept
    texts-cpt))

;; Construct initial pattern
;;
;; And
;;   preproof-of(A, T)
;;   preproof-of(B, T)
;;   expand(A, L, R, B)
;;
;; For now R is hardwired with
;;
;; DontExec
;;   DefinedSchema "conditional-full-instantiation-implication-scope-meta-rule"
(define (initpat)
  (LambdaLink
  (VariableList
    (VariableNode "$T")
    (VariableNode "$A")
    (VariableNode "$L")
    (VariableNode "$B")
  )
  (AndLink
    (preproof-of (List (DontExecLink (Variable "$A")) (Variable "$T")))
    (preproof-of (List (DontExecLink (Variable "$B")) (Variable "$T")))
    (expand
      (ListLink
        (DontExecLink (VariableNode "$A"))
        (VariableNode "$L")
        (DontExecLink
          (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule")
        )
      )
      (DontExecLink (VariableNode "$B"))
    )
  )
)
)
