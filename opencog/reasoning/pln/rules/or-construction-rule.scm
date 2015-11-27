; =====================================================================
; OrConstructionRule
; (http://wiki.opencog.org/w/OrConstructionRule TODO)
;
; For now A and B can be predicates or concepts. Note that the rule
; will not try to prevent mixing predicates and concepts (we need a
; better type system for that). Also it assumes that A and B are
; independant. The rule should account for relationships between A and
; B (like inheritance, etc) to correct that assumption.
;
; A<TV1>
; B<TV2>
; |-
; OrLink <TV>
;    A
;    B
;----------------------------------------------------------------------


(define or-construction-rule
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
        (GroundedSchemaNode "scm: or-construction-formula")
        (ListLink
           (VariableNode "$A")
           (VariableNode "$B")))))

(define (or-construction-formula A B)
  (cog-set-tv!
   (OrLink A B)
   (or-construction-side-effect-free-formula A B))
)

(define (or-construction-side-effect-free-formula A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B)))
    (stv (- (+ sA sB) (* sA sB)) (min cA cB))))

; Name the rule
(define or-construction-rule-name
  (Node "or-construction-rule"))
(DefineLink
   or-construction-rule-name
   or-construction-rule)
