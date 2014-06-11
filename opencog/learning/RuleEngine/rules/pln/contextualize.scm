; =====================================================================
; ContextualizeRule
; (http://wiki.opencog.org/w/ContextualizerRule)
; 
; a)
; R <TV>
;    C ANDLink A
;    C ANDLink B
; |-
; ContextLink <TV>
;    C
;    R A B
;----------------------------------------------------------------------

; This rule can be used to deal with inheritance relationships that apply
; only in certain circumstances and to differentiate between those cases.
; Example: "Ben is competent in the domain of mathematics.
;           Ben is not competent in the domain of juggling."
; (InheritanceLink
;   (AndLink
;       (ConceptNode "Ben")
;       (ConceptNode "doing_mathematics"))
;   (ConceptNode "competent"))
;
; (InheritanceLink
;   (AndLink
;       (ConceptNode "Ben")
;       (ConceptNode "juggling"))
;   (NotLink
;       (ConceptNode "competent")))
;
; Instances of Ben that are doing mathematics are competent, while
; instances of Ben that are juggling are incompetent. An AndLink 
; between two concepts designates the intersection of these concepts.
(define pln-rule-contextualize-inheritance
    (BindLink
        (ListLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
            (ImplicationLink
                (InheritanceLink
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$A"))
                    (AndLink
                        (VariableNode "$C")
                        (VariableNode "$B")))
                (ExecutionLink
                    (GroundedSchemaNode "scm: pln-formula-context")
                    (ListLink
                        ; this ContextLink is the desired output
                        (ContextLink
                            (VariableNode "$C")
                            (InheritanceLink
                                (VariableNode "$A")
                                (VariableNode "$B")))
                        ; the 2nd argument of pln-formula-context
                        (InheritanceLink
                            (AndLink
                                (VariableNode "$C")
                                (VariableNode "$A"))
                            (AndLink
                                (VariableNode "$C")
                                (VariableNode "$B"))))))))


; In an EvaluationLink, the PredicateNode is not 'andified'
; This rule is used specifically to deal with evaluations
; that differ between different contexts.
; Example: "In the context of the earth, the sky is blue.
; In the context of the earth, the sky is black."
(define pln-rule-contextualize-evaluation
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$A")
                (VariableTypeNode "PredicateNode"))
            (VariableNode "$B")
            (VariableNode "$C"))
        (ImplicationLink
            (EvaluationLink
                (VariableNode "$A")
                (AndLink
                    (VariableNode "$C")
                    (VariableNode "$B")))
            (ExecutionLink
                (GroundedSchemaNode "scm: pln-formula-context")
                (ListLink
                    (ContextLink
                        (VariableNode "$C")
                        (EvaluationLink
                            (VariableNode "$A")
                            (VariableNode "$B")))
                    (EvaluationLink
                        (VariableNode "$A")
                        (AndLink
                            (VariableNode "$C")
                            (VariableNode "$B"))))))))

;----------------------------------------------------------------------
; b)
; SubsetLink <TV>
;    C
;    A
; |-
; ContextLink <TV>
;    C
;    A
;----------------------------------------------------------------------

; This rule is used to transform a SubsetLink directly into a ContextLink.
; This is possible due to ConceptNode "X" (stv) being equivalent to
; (SubsetLink (stv)
;   (ConceptNode "Universe")
;   (ConceptNode "X"))
; Example: "A bulldog is a dog. Bulldogs appear in the context of dogs."
; (SubsetLink
;   (ConceptNode "dog")
;   (ConceptNode "bulldog"))
; |-
; (ContextLink
;   (ConceptNode "dog")
;   (ConceptNode "bulldog"))
(define pln-rule-contextualize-subset
    (BindLink
        (ListLink
            (VariableNode "$C")
            (VariableNode "$A"))
        (ImplicationLink
            (SubsetLink
                (VariableNode "$C")
                (VariableNode "$A"))
           (ExecutionLink
               (GroundedSchemaNode "scm:pln-formula-context")
               (ListLink
                   (ContextLink
                       (VariableNode "$C")
                       (VariableNode "$A"))
                   (SubsetLink
                       (VariableNode "$C")
                       (VariableNode "$A")))))))


(define (pln-formula-context Context Relation)
    (cog-set-tv! Context (cog-tv Relation)))
