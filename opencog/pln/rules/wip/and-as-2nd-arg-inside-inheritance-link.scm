;; =============================================================================
;; And as 2nd arg inside inheritance link rule
;;
;; InheritanceLink
;;   A
;;   B
;; InheritanceLink
;;   A
;;   C
;; |-
;; InheritanceLink
;;   A
;;   AndLink
;;       B
;;       C
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

(define and-as-2nd-arg-inside-inheritance-link-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: and-as-2nd-arg-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (AndLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (InheritanceLink
                    (VariableNode "$A")
                    (AndLink
                        (VariableNode "$B")
                        (VariableNode "$C")))))))

(define (and-as-2nd-arg-formula A B C AB AC BC ABC)
    (
        (cog-set-tv! 
            BC
            (stv 
                (* (cog-mean B) (cog-mean C))
                (min (cog-confidence B) (cog-confidence C))))
        (cog-set-tv!
            ABC
            (stv
                (/
                    (* 
                        (* (cog-mean AB) (cog-mean BC))
                        (* (cog-mean A) (cog-mean AC)))
                    (* (cog-mean C) (cog-mean B)))
                (min
                    (cog-confidence A)
                    (cog-confidence B)
                    (cog-confidence C)
                    (cog-confidence AB)
                    (cog-confidence AC))))))

