;===============================================================================
; ContextFreeToSensitiveRule
; http://wiki.opencog.org/w/ContextFreeToSensitiveRule
;
; C <TV1>
; A <TV2>
; |-
; ContextLink <TV3>
;     C
;     A
;-------------------------------------------------------------------------------

(define context-free-to-sensitive-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$C"))
        (AndLink
            (VariableNode "$C")
            (VariableNode "$A"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: context-free-to-sensitive-formula")
            (ListLink
                (ContextLink
                    (VariableNode "$C")
                    (VariableNode "$A"))
		(AndLink
                    (VariableNode "$C")
                    (VariableNode "$A"))))))

(define (context-free-to-sensitive-formula Context CA)
    (cog-set-tv! Context 
        (cog-new-stv (cog-mean CA) (cog-confidence CA))))
;            ; strength (now just computed as the mean of the strengths of C & A)
;           (/
;              (+
;                    (cog-mean C) (cog-mean A))
;                2)
;            ; confidence
;            (*
;                (cog-confidence C)
;                (cog-confidence A)
;                (- 1 (entropy (cog-mean C)))
;                (- 1 (entropy (cog-mean A))))

;(define (entropy p)
;    (-
;        (sum ; how should sigma be implemented here?
;             ; should a uniform distribution be assumed?
;            (*
;                p
;                (log p)))))

; Name the rule
(define context-free-to-sensitive-rule-name
  (DefinedSchemaNode "context-free-to-sensitive-rule"))
(DefineLink
  context-free-to-sensitive-rule-name
  context-free-to-sensitive-rule)
