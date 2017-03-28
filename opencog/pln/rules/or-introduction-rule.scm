;; To be replaced by fuzzy-disjunction-introduction-rule

; =====================================================================
; Or introduction rule
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


(define or-introduction-rule
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
        (GroundedSchemaNode "scm: or-introduction-formula")
        (ListLink
           (VariableNode "$A")
           (VariableNode "$B")))))

(define (or-introduction-formula A B)
  (cog-set-tv!
   (OrLink A B)
   (or-introduction-side-effect-free-formula A B))
)

(define (or-introduction-side-effect-free-formula A B)
  (let 
      ((sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (cA (cog-stv-confidence A))
       (cB (cog-stv-confidence B)))
    (stv (- (+ sA sB) (* sA sB)) (min cA cB))))

; Name the rule
(define or-introduction-rule-name
  (DefinedSchemaNode "or-introduction-rule"))
(DefineLink
   or-introduction-rule-name
   or-introduction-rule)
