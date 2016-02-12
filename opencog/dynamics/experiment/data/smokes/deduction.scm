; =============================================================================
; Deduction Rule
; TODO The rule must be applicable to ImplicationLink, SubsetLink and PartOfLink
;
; AND(Inheritance A B, Inheritance B C) entails Inheritance A C
; -----------------------------------------------------------------------------
(define pln-rule-deduction
    (BindLink
        (VariableList
                (TypedVariableLink
                   (VariableNode "$A")
                   (TypeNode "ConceptNode"))
                (TypedVariableLink
                   (VariableNode "$B")
                   (TypeNode "ConceptNode"))
                (TypedVariableLink
                   (VariableNode "$C")
                   (TypeNode "ConceptNode")))
        (AndLink
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B")
            )
            (InheritanceLink
                (VariableNode "$B")
                (VariableNode "$C")
            )
            ; To avoid matching (Inheritance A B) and (Inheritance B A)
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$C")
                )
            )
        )
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-simple-deduction")
            (ListLink
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C")
                )
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C")
                )
            )
        )
    )
)


; -----------------------------------------------------------------------------
; Deduction Formula
; -----------------------------------------------------------------------------

(define (pln-formula-simple-deduction AB BC AC)
    (let
        ((sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBC (cog-stv-strength BC))
         (cBC (cog-stv-confidence BC)))
      (if (and (>= sAB 0.5) (>= cAB 0.5) (>= sBC 0.5) (>= cBC 0.5))
          (cog-set-tv! AC (stv 1 1)))
    )
)

; Associate a name to the rule
(define pln-rule-deduction-name (DefinedSchemaNode "pln-rule-deduction"))
(DefineLink
  pln-rule-deduction-name
  pln-rule-deduction)
