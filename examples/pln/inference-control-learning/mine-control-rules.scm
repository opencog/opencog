(define (mine-control-rules)
  ;; Filter history, for now remove all root atoms with TV below
  ;; 0.1. That is because the pattern miner doesn't support values
  ;; yet.
  (remove-almost-false-atoms)

  ;; TODO: should do the same for almost true atoms, as to find
  ;; patterns predicting failure.

  (icl-logger-info "Mining control rules patterns")
  (for-each mine-control-rules-for (get-inference-rules)))

;; Like mine-control-rules but only mine the control rules for a given
;; inference rule
(define (mine-control-rules-for inference-rule)
  (icl-logger-info "Mining patterns for ~a" inference-rule)

  (let* ((texts-cpt (mk-texts inference-rule))
         (ipat (initpat inference-rule))
         (mined-patterns (cog-mine texts-cpt
                                   minsup
                                   #:maxiter maxiter
                                   #:initpat ipat)))
    (icl-logger-fine "Mined patterns = ~a" mined-patterns)))

;; Get all pln rules potential involved in the traces to mine
(define (get-inference-rules)
  (let* ((vardecl (VariableList
                     (TypedVariable (Variable "$A") (Type "DontExecLink"))
                     (TypedVariable (Variable "$B") (Type "DontExecLink"))
                     (TypedVariable (Variable "$R") (Type "DontExecLink"))
                     (Variable "$L")))
         (input (List
                  (Variable "$A")
                  (Variable "$L")
                  (Variable "$R")))
         (pattern (expand input (Variable "$B")))
         (bind (Bind vardecl pattern (Variable "$R")))
         (results (cog-execute! bind))
         (get-first-outgoing (lambda (x) (cog-outgoing-atom x 0))))
    (map get-first-outgoing (cog-outgoing-set results))))

;; Create a texts concept and add as members to it all atoms of the form
;;
;; preproof-of(A, T)
;; expand(A, L, R, B)
;;
;; where R is the given inference rule and A, T, L and B are variables
(define (mk-texts inference-rule)
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
                           (Variable "$L")))
         (expand-input (List
                         (Variable "$A")
                         (Variable "$L")
                         (DontExec inference-rule)))
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
(define (initpat inference-rule)
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
        (DontExecLink inference-rule)
      )
      (DontExecLink (VariableNode "$B"))
    )
  )
)
)
