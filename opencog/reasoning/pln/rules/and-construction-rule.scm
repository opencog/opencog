; =====================================================================
; AndRule
; (http://wiki.opencog.org/w/AndRule TODO)
;
; A<TV1>
; B<TV2>
; |-
; AndLink <TV>
;    A
;    B
;----------------------------------------------------------------------


(define pln-rule-and-construction
  (BindLink
     (VariableList
        (TypedVariableLink
           (VariableNode "$A")
           (TypeChoice
              (TypeNode "PredicateNode")
              (TypeNode "ConceptNode")))
        (TypedVariableLink
           (VariableNode "$B")
           (TypeChoice
              (TypeNode "PredicateNode")
              (TypeNode "ConceptNode"))))
     (AndLink
        (VariableNode "$A")
        (VariableNode "$B")
        (NotLink
           (EqualLink
              (VariableNode "$A")
              (VariableNode "$B"))))
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-and-construction")
        (ListLink
           (VariableNode "$A")
           (VariableNode "$B")))))

(define (pln-formula-and-construction A B)
  (cog-set-tv!
   (AndLink A B)
   (pln-formula-and-side-effect-free A B))
)

(define (pln-formula-and-side-effect-free A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B)))
    (stv (* sA sB) (min cA cB))))

;; ; Name the rule
;; (define pln-rule-and-construction-name
;;   (Node "pln-rule-and-construction"))
;; (DefineLink
;;    pln-rule-and-construction-name
;;    pln-rule-and-construction)
