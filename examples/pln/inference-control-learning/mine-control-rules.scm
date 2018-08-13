(define (mine-control-rules)
  (let* ((mine-as (cog-new-atomspace))
         (old-as (cog-set-atomspace! mine-as)))
    ;; Copy history-as to the mining atomspace
    (cp-as history-as mine-as)

    ;; Filter history, for now remove all root atoms with TV below
    ;; 0.1. That is because the pattern miner doesn't support values
    ;; yet.
    (remove-almost-false-atoms)

    ;; TODO: should do the same for almost true atoms, as to find
    ;; patterns predicting failure.

    ;; Mine mine-as for control rules
    (let* ((inference-rules (get-inference-rules history-as))
           (ctrl-rules-lsts (map mine-control-rules-for inference-rules))
           (ctrl-rules (apply append ctrl-rules-lsts)))

      (icl-logger-fine "Mined control rules = ~a" ctrl-rules)

      ;; Copy the control rules in the previous atomspace
      (icl-cp old-as ctrl-rules))

    ;; Pop mine-as
    (cog-set-atomspace! old-as)))

;; Like mine-control-rules but only mine the control rules for a given
;; inference rule
(define (mine-control-rules-for inference-rule)
  (icl-logger-debug "Mining patterns for ~a" inference-rule)

  (let* ((texts-cpt (mk-texts inference-rule))
         (ipat (initpat inference-rule))
         (patterns (cog-mine texts-cpt
                             minsup
                             #:maxiter miter
                             #:initpat ipat))
         (patterns-lst (cog-outgoing-set patterns))
         (ctrl-rules (map pattern->ctrl-rule patterns-lst)))
    ;; (icl-logger-fine "Mined patterns-lst = ~a" patterns-lst)
    ;; (icl-logger-fine "Mined ctrl-rules = ~a" ctrl-rules)
    ctrl-rules))

;; Takes a pattern like
;;
;; Lambda
;;   VariableList T A L B
;;   And
;;     preproof-of(DontExec A, T)
;;     preproof-of(DontExec B, T)
;;     expand(DontExec A, L, R, DontExec B)
;;
;; and turns into a control rule like
;;
;; ImplicationScope
;;   VariableList
;;     T
;;     TypedVariable A' DontExec
;;     L
;;     TypedVariable B' DontExec
;;   And
;;     preproof-of(A', T)
;;     expand(A', L, R, B')
;;   preproof-of(B', T)
;;
;; NEXT TODO: take care of the DontExec parts
(define (pattern->ctrl-rule pattern)
  (let* (;; Get vardecl
         (vardecl (gar pattern))
         (clauses (cog-outgoing-set (gdr pattern)))

         ;; Get expand clause
         (expand-AB? (lambda (x) (equal? (cog-type x) 'ExecutionLink)))
         (expand-AB (find expand-AB? clauses))
         (preproof-clauses (remove expand-AB? clauses))

         ;; Get preproof A clause
         (A (cog-outgoing-atom (cog-outgoing-atom expand-AB 1) 0))
         (preproof-A? (lambda (x) (equal? A (get-inference x))))
         (preproof-A (find preproof-A? preproof-clauses))

         ;; Get preproof B clause
         (B (cog-outgoing-atom expand-AB 2))
         (preproof-B? (lambda (x) (equal? B (get-inference x))))
         (preproof-B (find preproof-B? preproof-clauses))

         ;; Rearrenge into implication scope
         (antecedent (And preproof-A expand-AB))
         (consequent preproof-B)
         (implication (ImplicationScope vardecl antecedent consequent)))
    implication))

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

    ;; (icl-logger-fine "mk-texts members of texts-cpt = ~a" (get-members texts-cpt))

    ;; Return texts concept
    texts-cpt))

;; Construct initial pattern
;;
;; Lambda
;;   VariableList T A L B
;;   And
;;     preproof-of(A, T)
;;     preproof-of(B, T)
;;     expand(A, L, R, B)
;;
;; where R is inference-rule
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
