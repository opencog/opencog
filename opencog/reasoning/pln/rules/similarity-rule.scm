;; =============================================================================
;; Similarity Rule
;;
;; AndLink
;;   A
;;   B
;; OrLink
;;   A
;;   B
;; |-
;; SimilarityLink
;;   A
;;   B
;;
;; -----------------------------------------------------------------------------

(define similarity-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (OrLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: similarity-formula")
            (ListLink
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (OrLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SimilarityLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

(define (similarity-formula AAB OAB SAB)
    (cog-set-tv! 
        SAB
        (if 
            (= (* (cog-stv-strength OAB) (cog-stv-confidence OAB)) 0)
            (stv 0 0)
            (stv (/
                    (* (cog-stv-strength AAB) (cog-stv-confidence AAB))
                    (* (cog-stv-strength OAB) (cog-stv-confidence OAB)))
                (min (cog-stv-confidence OAB) (cog-stv-confidence AAB))))))

