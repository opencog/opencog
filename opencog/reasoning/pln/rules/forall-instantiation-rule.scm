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
           (VariableNode "$TyV")
           (TypeNode "TypedVariableLink"))
        (VariableNode "$B"))
     (ForAllLink
        (VariableNode "$TyV")
        (VariableNode "$B"))
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-forall-instantiation")
        (ListLink
           (VariableNode "$TyV")
           (VariableNode "$B")))))

; This function
; 1. Perform the substitution
; 2. Calculate its TV (TODO: just <1 1> for now)
;
; Warning: if there is more than one variable in B this is gonna
; crash! (that is because PutLink expect the number of variables in
; the ListLink to be equal to the number of free variables in the
; body)
(define (pln-formula-forall-instantiation TyV B)
  (cog-set-tv!
   (let* (
                                        ; TyV corresponds to something like
                                        ;
                                        ; (TypedVariableLink
                                        ;    (VariableNode "$V")
                                        ;    (TypeNode "TypedVariableLink"))
                                        ;
                                        ; Take the VariableNode "$V"
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

     ; Substitute the variable by the term in the body
     (cog-execute! (PutLink B term)))
   (stv 1 1)))

;; ; Name the rule
(define pln-rule-forall-instantiation-name
  (Node "pln-rule-forall-instantiation-name"))
(DefineLink pln-rule-forall-instantiation-name
  pln-rule-forall-instantiation)
