;; =============================================================================
;; And as 1st arg inside inheritance link rule
;;
;; InheritanceLink
;;   A
;;   C
;; InheritanceLink
;;   B
;;   C
;; |-
;; InheritanceLink
;;   AndLink
;;       A
;;       B
;;   C
;;
;; -----------------------------------------------------------------------------
(load "formulas.scm")

(define and-as-1st-arg-inside-inheritance-link-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (InheritanceLink
                (VariableNode "$B")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: and-as-1st-arg-formula")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (VariableNode "$C"))))))

(define (and-as-1st-arg-formula A B C AC BC AB ABC)
    (
        (cog-set-tv! 
            AB 
            (stv 
                (* (cog-mean A) (cog-mean B))
                (min (cog-confidence A) (cog-confidence B))))
        (cog-set-tv!
            ABC
            (stv
                (/
                    (* 
                        (* (cog-mean AC) (cog-mean BC))
                        (* (cog-mean A) (cog-mean B)))
                    (* (cog-mean C) (cog-mean AB)))
                (min 
                    (cog-confidence A)
                    (cog-confidence B)
                    (cog-confidence C)
                    (cog-confidence AC)
                    (cog-confidence BC))))))

