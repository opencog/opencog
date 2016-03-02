; Copyright (C) 2016 OpenCog Foundation

;; --------------------------------------------------------------
;; Helper function
(define (eg-demand-values result)
"
  Return the demand-values of the demands passed to it from `(psi-step)`.
"

    (map (lambda (x) (tv-mean (cog-tv x))) (cog-outgoing-set result))

)

; --------------------------------------------------------------
(define (step-1-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.67449999 0.99999982))
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        '()
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Default")
              (ConceptNode "OpenPsi: Energy" (stv 0.67449999 0.99999982))
           )
        )
        '()
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.67449999 0.99999982))
        )
    )
)


(define (step-1-dv) (list 0.6744999885559082))


; --------------------------------------------------------------
(define (step-2-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.64077497 0.99999982))
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        '()
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Default")
              (ConceptNode "OpenPsi: Energy" (stv 0.64077497 0.99999982))
           )
        )
        '()
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.64077497 0.99999982))
        )
    )
)

(define (step-2-dv) (list 0.6407749652862549))

; --------------------------------------------------------------
(define (step-3-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.66960984 0.99999982))
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        '()
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Increase")
              (ConceptNode "OpenPsi: Energy" (stv 0.66960984 0.99999982))
           )
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10"))
        (list
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default")
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
        )
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.66960984 0.99999982))
        )
    )
)

(define (step-3-dv) (list 0.6696098446846008))

; --------------------------------------------------------------
(define (step-4-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.63612938 0.99999982))
        )
        (list
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default")
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
        )
        (ConceptNode "OpenPsi: Energy" (stv 0.63612938 0.99999982))
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Default")
              (ConceptNode "OpenPsi: Energy" (stv 0.63612938 0.99999982))
           )
        )
        '()
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.63612938 0.99999982))
        )
    )
)

(define (step-4-dv) (list 0.6361293792724609))

; --------------------------------------------------------------
(define (step-5-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.66475523 0.99999982))
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        '()
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Increase")
              (ConceptNode "OpenPsi: Energy" (stv 0.66475523 0.99999982))
           )
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10"))
        (list
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default")
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
        )
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.66475523 0.99999982))
        )
    )
)

(define (step-5-dv) (list 0.6647552251815796))

; --------------------------------------------------------------
(define (step-6-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.63151747 0.99999982))
        )
        (list
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default")
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
        )
        (ConceptNode "OpenPsi: Energy" (stv 0.63151747 0.99999982))
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Default")
              (ConceptNode "OpenPsi: Energy" (stv 0.63151747 0.99999982))
           )
        )
        '()
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.63151747 0.99999982))
        )
    )
)

(define (step-6-dv) (list 0.6315174698829651))

; --------------------------------------------------------------
(define (step-7-result)
    (list
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.65993577 0.99999982))
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
        '()
        (StateLink
           (Node "OpenPsi: action-on-demand")
           (ListLink
              (ConceptNode "OpenPsi: Increase")
              (ConceptNode "OpenPsi: Energy" (stv 0.65993577 0.99999982))
           )
        )
        (list (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10"))
        (list
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default")
            (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
        )
        (SetLink
           (ConceptNode "OpenPsi: Energy" (stv 0.65993577 0.99999982))
        )
    )
)

(define (step-7-dv) (list 0.6599357724189758))
