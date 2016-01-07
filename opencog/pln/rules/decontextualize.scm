; =====================================================================
; DecontextualizeRule
; http://wiki.opencog.org/w/DecontextualizerRule
; 
; a)
; ContextLink <TV>
;     C
;     R A B
; |-
; R <TV>
;     C ANDLink A
;     C ANDLink B
;----------------------------------------------------------------------

; Inverse of (contextualize-inheritance-rule):
; a ContextLink consisting of an InheritanceLink is decontextualized,
; i.e. reduced to an inheritance relationship
(define decontextualize-inheritance-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (ContextLink
            (VariableNode "$C")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: context-formula")
            (ListLink
                (InheritanceLink
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$A"))
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B")))
                (ContextLink
                    (VariableNode "$C")
                    (InheritanceLink
                        (VariableNode "$A")
                        (VariableNode "$B")))))))

; Inverse of (contextualize-evaluation-rule):
; in an EvaluationLink, the PredicateNode is not 'andified';
; gets rid of the ContextLink enclosing an EvaluationLink
(define decontextualize-evaluation-rule
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$A")
                (TypeNode "PredicateNode"))
            (VariableNode "$B")
            (VariableNode "$C"))
        (ContextLink
            (VariableNode "$C")
            (EvaluationLink
                (VariableNode "$A")
                (ListLink
                    (VariableNode "$B"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: context-formula")
            (ListLink
                (EvaluationLink
                    (VariableNode "$A")
                    (ListLink
                        (AndLink
                            (VariableNode "$C")
                            (VariableNode "$B"))))
                (ContextLink
                    (VariableNode "$C")
                    (EvaluationLink
                        (VariableNode "$A")
                        (ListLink
                            (VariableNode "$B"))))))))

;----------------------------------------------------------------------
; b)
; ContextLink <TV>
;     C
;     A
; |-
; SubsetLink <TV>
;     C
;     A
;----------------------------------------------------------------------
; Inverse of (contextualize-subset-rule):
; ContextLink is transformed into a SubsetLink
(define decontextualize-subset-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$C"))
        (ContextLink
            (VariableNode "$C")
            (VariableNode "$A"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: context-formula")
            (ListLink
                (SubsetLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
                (ContextLink
                    (VariableNode "$C")
                    (VariableNode "$A"))))))

(define (context-formula Relation Context)
    (cog-set-tv! Relation (cog-tv Context)))

; Name the rule
(define contextualize-inheritance-rule-name
  (DefinedSchemaNode "contextualize-inheritance-rule")
(DefineLink
  contextualize-inheritance-rule-name
  contextualize-inheritance-rule)
