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

;; Only try to match a ForAllLink with a type restricted variable in
;; the ForAllLink variable definition. The choice of the substitution
;; term is done randomly in pln-formula-forall-instantiation.
(define pln-rule-forall-instantiation
  (BindLink
     (VariableList
        (TypedVariableLink
           (VariableNode "$SV")
           (TypeChoice
              (TypeNode "TypedVariableLink")
              (TypeNode "VariableList")))
        (VariableNode "$B"))
     (ForAllLink
        (VariableNode "$SV")
        (VariableNode "$B"))
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-forall-instantiation")
        (ListLink
           (VariableNode "$SV")
           (VariableNode "$B")))))

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
    (apply ListLink terms))
)

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
         (result (cog-execute! query))
                                        ; Get the list of them
         (terms (cog-outgoing-set result))
                                        ; Select one randomly
         (term (list-ref terms (random (length terms)))))
    term)
)

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
(define (pln-formula-forall-instantiation SV B)
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

;; ; Name the rule
(define pln-rule-forall-instantiation-name
  (Node "pln-rule-forall-instantiation-name"))
(DefineLink pln-rule-forall-instantiation-name
  pln-rule-forall-instantiation)
