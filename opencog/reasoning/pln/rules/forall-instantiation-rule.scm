; =======================================================================
; ForAll Instantiation rule
; (TODO add wiki page)
;
; ForAllLink
;    V
;    B
; T
; |-
; B[V->T]
;
; where V is a variable, B is a body containing V, T is an atom to
; substitute and B[V->T] is B where V has been substituted by T
;
; Contains as well a version with 3 universally quantified variables.
; -----------------------------------------------------------------------

(use-modules (opencog exec))
(use-modules (opencog logger))

;;;;;;;;;;;;;;;;;;;;;;;
;; Helper definition ;;
;;;;;;;;;;;;;;;;;;;;;;;

(define forall-variable-pattern
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$B")))

(define forall-body-pattern
  (ForAllLink
     (VariableNode "$TyVs")
     (VariableNode "$B")))

; Helper for pln-formula-forall-instantiation. Given an atom
;
; (VariableList
;    (TypedVariableLink
;       (VariableNode "$V1")
;       <type-constraint1>)
;    ...
;    (TypedVariableLink
;       (VariableNode "$Vn")
;       <type-constraintn>))
;
; return a ListLink of random terms satisfying the type constraints of
; V1 to Vn.
(define (select-substitution-terms TyVs)
  (let* (
                                        ; Get the list of typed
                                        ; variables V1 to Vn
         (typed-vars (cog-outgoing-set TyVs))
                                        ; Pick up a random term of each
         (terms (map select-substitution-term typed-vars)))
    (apply ListLink terms)))

; Select a random atom from a Link's outgoings
(define (select-rnd-outgoing link)
  (let ((outgoings (cog-outgoing-set link)))
    (list-ref outgoings (random (length outgoings)))))

; Helper for pln-formula-forall-instantiation. Given an atom
;
; (TypedVariableLink
;    (VariableNode "$V")
;    <type-constraint>)
;
; return a random term satisfying the type constraint.
(define (select-substitution-term TyV)
  (let* (
         (V (gar TyV))
                                        ; Build pattern matcher query
                                        ; for the subtitution term
         (query (GetLink TyV V))
                                        ; Fetch all possible substitution terms
         (result (cog-execute! query)))
                                        ; Select one randomly
    (select-rnd-outgoing result)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Forall full instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define forall-full-instantiation-rewrite-rule
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: pln-formula-forall-full-instantiation")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$B"))))

;; Only try to match a ForAllLink with a type restricted variable in
;; the ForAllLink variable definition. The choice of the substitution
;; term is done randomly in pln-formula-forall-full-instantiation. All
;; scoped variables are instantiated.
(define pln-rule-forall-full-instantiation
  (BindLink
     forall-variable-pattern
     forall-body-pattern
     forall-full-instantiation-rewrite-rule))

; This function
;
; 1. randomly selects a substitution term (or a tuple of substitution
;    terms, if the ForAll has multiple variables in scope),
;
; 2. performs the substitution,
;
; 3. calculates its TV (TODO: just <1 1> for now).
;
; Warning: there must be the same number of free variables in the body
; and scoped variables in the forall, otherwise this is gonna crash.
; That is because PutLink expects the number of variables in the
; ListLink to be equal to the number of free variables in the body.
(define (pln-formula-forall-full-instantiation SV B)
  (cog-set-tv!
   (let* (
          (SV-type (cog-type SV))
                                        ; Build a list of all type
                                        ; contrained variables
                                        ;
                                        ; There is only one type
                                        ; constrained variable
          (terms (cond ((cog-subtype? 'TypedVariableLink SV-type)
                        (select-substitution-term SV))
                                        ; there are multiple type
                                        ; constrained variables
                       ((cog-subtype? 'VariableList SV-type)
                        (select-substitution-terms SV))
                       (cog-logger-error
                        (string-append "Wrong type for ~a, "
                                       "should be a TypedVariableLink "
                                       "or a VariableList") SV))))
     
     ; Substitute the variable by the term in the body
     (cog-execute! (PutLink (LambdaLink SV B) terms)))
   (stv 1 1)))

;; Name the rule
(define pln-rule-forall-full-instantiation-name
  (Node "pln-rule-forall-full-instantiation-name"))
(DefineLink pln-rule-forall-full-instantiation-name
  pln-rule-forall-full-instantiation)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Forall partial instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define forall-partial-instantiation-rewrite-rule
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: pln-formula-forall-partial-instantiation")
     (ListLink
        (VariableNode "$TyVs")
        (VariableNode "$B"))))

;; Like pln-rule-forall-full-instantiation but only instantiate one variable
(define pln-rule-forall-partial-instantiation
  (BindLink
     forall-variable-pattern
     forall-body-pattern
     forall-partial-instantiation-rewrite-rule))

; This function
;
; 1. randomly selects a substitution term (or a tuple of substitution
;    terms, if the ForAll has multiple variables in scope),
;
; 2. performs the substitution,
;
; 3. calculates its TV (TODO: just <1 1> for now).
;
; Warning: there must be the same number of free variables in the body
; and scoped variables in the forall, otherwise this is gonna crash.
; That is because PutLink expects the number of variables in the
; ListLink to be equal to the number of free variables in the body.
(define (pln-formula-forall-partial-instantiation TyVs B)
  (cog-set-tv!
   (let* (
          (TyVs-type (cog-type TyVs))
                                        ; Select the variable to substitute
          (TyV (cond ((cog-subtype? 'TypedVariableLink TyVs-type) TyVs)
                     ((cog-subtype? 'VariableList TyVs-type)
                      (select-rnd-outgoing TyVs))
                     (cog-logger-error
                      (string-append "Wrong type for ~a, "
                                     "should be a TypedVariableLink "
                                     "or a VariableList") TyVs)))
          (term (select-substitution-term TyV)))

     ; Substitute the variable by the term in the body
     (cog-execute! (PutLink (LambdaLink TyV B) term)))
   (stv 1 1)))

;; Name the rule
(define pln-rule-forall-partial-instantiation-name
  (Node "pln-rule-forall-partial-instantiation-name"))
(DefineLink pln-rule-forall-partial-instantiation-name
  pln-rule-forall-partial-instantiation)
