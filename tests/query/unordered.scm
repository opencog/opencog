
(define (stv mean conf) (cog-new-stv mean conf))

; SENTENCE: [The color of the sky is blue.]
; _predadj (<<color>>, <<blue>>) 
(EvaluationLink (stv 1.0 1.0)
   (DefinedLinguisticRelationshipNode "_predadj")
   (ListLink
      (WordInstanceNode "color@8947b41d-b6d8-44c5-95a0-a909220596d5")
      (WordInstanceNode "blue@cf040834-cf7a-42ae-bd42-83a001d3c3e3")
   )
)
; of (<<color>>, <<sky>>) 
(EvaluationLink (stv 1.0 1.0)
   (PrepositionalRelationshipNode "of")
   (ListLink
      (WordInstanceNode "color@8947b41d-b6d8-44c5-95a0-a909220596d5")
      (WordInstanceNode "sky@d30ab6dd-0785-4b14-8969-f23697d384a7")
   )
)


; The whole point here is that the NotLink means that no match at all
; should be found. i.e. $prep can match "of" and should thus be
; rejected.
;

(define (blink)
   (BindLink
      (ListLink 
         (TypedVariableLink
            (VariableNode "$var2")
            (VariableTypeNode "WordInstanceNode")
         )
         (TypedVariableLink
            (VariableNode "$prep")
            (VariableTypeNode "PrepositionalRelationshipNode")
         )
         (TypedVariableLink
             (VariableNode "$var3")
             (VariableTypeNode "WordInstanceNode")
         )
         (VariableNode "$var1")
      )
      (ImplicationLink
         (AndLink
            (EvaluationLink (stv 1 0.99999988)
               (DefinedLinguisticRelationshipNode "_predadj")
               (ListLink 
                  (VariableNode "$var2")
                  (VariableNode "$var1")
               )
            )
            (NotLink 
               (EvaluationLink (stv 1 0.99999988) 
                  (VariableNode "$prep")
                  (ListLink 
                     (VariableNode "$var2")
                     (VariableNode "$var3")
                  )
               )
            )
         )
         (ListLink (stv 1 0.99999988)
            (VariableNode "$var1")
            (VariableNode "$var2")
            (VariableNode "$var3")
            (VariableNode "$prep")
         )
      )
   )
)

