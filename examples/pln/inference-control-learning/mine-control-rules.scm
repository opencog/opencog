(define (mine-all-control-rules)
  ;; Mine positive control rules
  (mine-control-rules #f)
  ;; Mine negative control rules
  (mine-control-rules #t)
)

(define (mine-control-rules negative)
  (if negative (icl-logger-debug "Mine negative control rules")
      (icl-logger-debug "Mine positive control rules"))

  (let* ((mine-as (cog-new-atomspace))
         (old-as (cog-set-atomspace! mine-as)))
    ;; Copy history-as to the mining atomspace
    (cp-as history-as mine-as)

    (if negative
        ;; Wrap Not around almost false preproof-of(A, T) atoms
        (wrap-not-almost-false-preproof-of)
        ;; Filter history, for now remove all root atoms with TV below
        ;; 0.1. That is because the pattern miner doesn't support values
        ;; yet.
        (remove-almost-false-atoms))

    ;; Mine mine-as for control rules
    (let* ((inference-rules (get-inference-rules history-as))
           (mcrf (lambda (x) (mine-control-rules-for x negative)))
           (ctrl-rules-lsts (map mcrf inference-rules))
           (ctrl-rules (apply append ctrl-rules-lsts)))

      (icl-logger-fine "Mined control rules = ~a" ctrl-rules)

      ;; Copy the control rules in the previous atomspace
      (icl-cp old-as ctrl-rules))

    ;; Pop mine-as
    (cog-set-atomspace! old-as)))

;; Like mine-negative-control-rules but only mine the control rules
;; for a given inference rule
(define (mine-control-rules-for inference-rule negative)
  (icl-logger-fine "Mining patterns for ~a" inference-rule)

  (let* ((texts-cpt (mk-texts inference-rule negative))
         (ipat (initpat inference-rule negative))
         (patterns (cog-mine texts-cpt
                             minsup
                             #:maximum-iterations miter
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
(define (pattern->ctrl-rule pattern)
  ;; (icl-logger-debug "pattern->ctrl-rule pattern = ~a" pattern)
  (let* (;; Get vardecl
         (vardecl (gar pattern))
         (clauses (map rm-dontexec (cog-outgoing-set (gdr pattern))))

         ;; Get expand clause
         (expand-AB? (lambda (x) (equal? (cog-type x) 'ExecutionLink)))
         (expand-AB (find expand-AB? clauses))
         (preproof-clauses (remove expand-AB? clauses))

         ;; Get preproof A clause
         (A (cog-outgoing-atom (cog-outgoing-atom expand-AB 1) 0))
         (preproof-A? (lambda (x) (equal? A (preproof-of->inference x))))
         (preproof-A (find preproof-A? preproof-clauses))

         ;; Get preproof B clause
         (B (cog-outgoing-atom expand-AB 2))
         (preproof-B? (lambda (x) (equal? B (preproof-of->inference x))))
         (not-preproof-B? (lambda (x) (equal? (cog-type x) 'NotLink)))
         (preproof-B-or-not? (lambda (x) (or (preproof-B? x)
                                             (not-preproof-B? x))))
         (preproof-B-or-not (find preproof-B-or-not? preproof-clauses))
         (preproof-B (if (not-preproof-B? preproof-B-or-not)
                         (cog-outgoing-atom preproof-B-or-not 0)
                         preproof-B-or-not))

         ;; Typed A and B with DontExecLink
         (de-vardecl (dontexec-vardecl vardecl A B))

         ;; Rearrenge into implication scope
         (antecedent (And preproof-A expand-AB))
         (consequent preproof-B)
         (implication (ImplicationScope
                        de-vardecl
                        antecedent
                        consequent)))
    implication))

;; Take a variable declaration, (DontExec (Variable A)) and (DontExec
;; (Variable B) and return that save variable declaration where A and
;; B are typed DontExecLink.
(define (dontexec-vardecl vardecl A B)
  (let* ((out (cog-outgoing-set vardecl))
         (A? (lambda (x) (equal? x A)))
         (B? (lambda (x) (equal? x B)))
         (dontexec-type-if (lambda (x) (cond ((A? x) (dontexec-typed A))
                                             ((B? x) (dontexec-typed B))
                                             (#t x))))
         (dontexec-out (map dontexec-type-if out)))
    (VariableList dontexec-out)))

;; Return #t iff x is of the form (DontExec (Variable ...))
(define (dontexec-variable? x)
  (and (equal? (cog-type x) 'DontExecLink)
       (equal? (cog-type (cog-outgoing-atom x 0)) 'VariableNode)))

;; Recursively remove DontExec links from atom
(define (rm-dontexec x)
  (let* ((x-type (cog-type x))
         (x-out (cog-outgoing-set x)))
    (cond ((cog-node? x) x)
          ((dontexec-variable? x) (car x-out))
          (#t (let* ((rde-x-out (map rm-dontexec x-out)))
                (cog-new-link x-type rde-x-out))))))

;; Create a texts concept and add as members to it all atoms of the form
;;
;; preproof-of(A, T)
;; expand(A, L, R, B)
;; preproof-of(B, T)
;;
;; or, if negative is #t
;;
;; preproof-of(A, T)
;; expand(A, L, R, B)
;; Not
;;   preproof-of(B, T)
;;
;; where R is the given inference rule and A, T, L and B are variables
(define (mk-texts inference-rule negative)
  (icl-logger-fine-atomspace (cog-atomspace))
  (let* (;; Create texts concept node
         (texts-cpt (random-node 'ConceptNode 8 "texts-"))

         ;; Build vardecl
         (T (Variable "$T"))
         (A (Variable "$A"))
         (L (Variable "$L"))
         (B (Variable "$B"))
         (vardecl (VariableList
                    T
                    (dontexec-typed A)
                    L
                    (dontexec-typed B)))

         ;; preproof-of(A, T)
         (preproof-A-clause (preproof-of (List A T)))
         (preproof-A-precondition (absolutely-true-eval preproof-A-clause))

         ;; [Not] preproof-of(B, T)
         (preproof-B-clause (if negative (Not (preproof-of (List B T))) (preproof-of (List B T))))
         (preproof-B-precondition (absolutely-true-eval preproof-B-clause))

         ;; expand(A, L, R, B)
         (expand-input (List
                         A
                         L
                         (DontExec inference-rule)))
         (expand-clause (expand expand-input B))
         (expand-precondition (absolutely-true-eval expand-clause))

         ;; preproof-of(A, T) & expand(A, L, R, B) & [Not] preproof-of(B, T)
         (pattern (And preproof-A-clause
                       preproof-B-clause
                       expand-clause
                       preproof-A-precondition
                       preproof-B-precondition
                       expand-precondition))

         ;; Fetch preproof-of(A, T)
         (preproof-A-bind (Bind vardecl pattern preproof-A-clause))
         (preproof-A-results (cog-execute! preproof-A-bind))

         ;; Fetch [Not] preproof-of(B, T)
         (preproof-B-bind (Bind vardecl pattern preproof-B-clause))
         (preproof-B-results (cog-execute! preproof-B-bind))

         ;; Fetch expand(A, L, R, B)
         (expand-bind (Bind vardecl pattern expand-clause))
         (expand-results (cog-execute! expand-bind))

         ;; Define function to add a member to texts-cpt
         (add-text (lambda (x) (Member x texts-cpt))))

    ;; Add all fetched preproof-of as members of texts-cpt
    (for-each add-text (cog-outgoing-set preproof-A-results))

    ;; Add all fetched Not preproof-of as members of texts-cpt
    (for-each add-text (cog-outgoing-set preproof-B-results))

    ;; Add all fetched expand as members of texts-cpt
    (for-each add-text (cog-outgoing-set expand-results))

    (let* ((members (get-members texts-cpt))
           (size (length members)))
      (icl-logger-fine "mk-texts: size = ~a" size)
      (icl-logger-fine "mk-texts: members = ~a" members))

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
;; where R is inference-rule.
;;
;; If negative is #t then preproof-of(B, T) is wrapped with a NotLink.
(define (initpat inference-rule negative)
  (let* ((T (VariableNode "$T"))
         (A (VariableNode "$A"))
         (L (VariableNode "$L"))
         (B (VariableNode "$B"))
         (vardecl (VariableList T A L B))
         (preproof-of-A-T (preproof-of (List (DontExecLink A) T)))
         (preproof-of-B-T (preproof-of (List (DontExecLink B) T)))
         (expand-A-L-R-B (expand
                           (ListLink
                             (DontExecLink A)
                             L
                             (DontExecLink inference-rule))
                           (DontExecLink B))))
    (LambdaLink
      vardecl
      (AndLink
        preproof-of-A-T
        (if negative
            (Not preproof-of-B-T)
            preproof-of-B-T)
        expand-A-L-R-B))))

(define (wrap-not-almost-false-preproof-of)
  (wrap-not-almost-false-preproof-of-atom-list (get-all-atoms)))

(define (wrap-not-almost-false-preproof-of-atom-list atoms)
  (for-each wrap-not-almost-false-preproof-of-atom atoms))

(define (almost-false-preproof-of? atom)
  (and (almost-null-strength? atom)
       (preproof-of? atom)))

(define (wrap-not-almost-false-preproof-of-atom atom)
  (if (and (cog-atom? atom)
           (not (null-confidence? atom))
           (almost-false-preproof-of? atom))
      (Not (stv 1 1) atom)))            ; TODO it should be (stv 1 1)
                                        ; but that methodology is wrong
                                        ; anyawy. Instead it should
                                        ; mine observed-preproof-of or
                                        ; something, that way we dont't
                                        ; need to worry almost false
                                        ; things.
