; Copyright (C) 2015 OpenCog Foundation

(load-from-path "openpsi/demand/utilities")

(define psi-demand-updater-rule
    (BindLink
        (VariableList (assoc-ref (psi-demand-pattern) "var"))
        (AndLink (assoc-ref (psi-demand-pattern) "pat"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: psi-demand-updater") ; pre-condition
            (ListLink
                (VariableNode "Demand")
                (VariableNode "min_acceptable_value")
                (VariableNode "max_acceptable_value")))))

(define (psi-demand-updater demand min-acceptable-value max-acceptable-value)
    (let ((current-value (tv-mean (cog-tv  demand)))
          (min-value (string->number (cog-name min-acceptable-value)))
          (max-value (string->number (cog-name max-acceptable-value)))
         )
         ; TODO:
         ;1. check the 'fuzzy_within' equations, it seems too have small effect.
         ;2. make it easier for different demands have differing updaters, as
         ;   different characters controled are likely have different
         ;   personality.
         (cog-set-tv! demand
             (stv (fuzzy_within current-value min-value max-value 100) 1)
         )
     )
     #t
)
