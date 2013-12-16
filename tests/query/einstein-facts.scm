;;
;; einstein-facts.scm
;;
;; Basic facts for the Einstein puzzle
;;

(define (stv mean conf) (cog-new-stv mean conf))


; Base concepts
(ConceptNode "house" (stv 0.05 1))
(ConceptNode "person" (stv 0.05 1))
(ConceptNode "pet" (stv 0.05 1))
(ConceptNode "drink" (stv 0.05 1))
(ConceptNode "cigaretteBrand" (stv 0.05 1))
(ConceptNode "colour" (stv 0.05 1))


; Colours
(EvaluationLink (stv 1 1)
   (PredicateNode "is_color")
   (ListLink
      (ConceptNode "red")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_color")
   (ListLink
      (ConceptNode "green")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_color")
   (ListLink
      (ConceptNode "white")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_color")
   (ListLink
      (ConceptNode "yellow")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_color")
   (ListLink
      (ConceptNode "blue")
      (ConceptNode "true")
   )
)


; nationality
(EvaluationLink (stv 1 1)
   (PredicateNode "is_nationality")
   (ListLink
      (ConceptNode "British")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_nationality")
   (ListLink
      (ConceptNode "Swedish")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_nationality")
   (ListLink
      (ConceptNode "Danish")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_nationality")
   (ListLink
      (ConceptNode "Norwegian")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_nationality")
   (ListLink
      (ConceptNode "German")
      (ConceptNode "true")
   )
)


; Pets
(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "dog")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "bird")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "cat")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "horse")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_pet")
   (ListLink
      (ConceptNode "fish")
      (ConceptNode "true")
   )
)


; Cigarettes

(EvaluationLink (stv 1 1)
   (PredicateNode "is_cigaretteBrand")
   (ListLink
      (ConceptNode "pallmall")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_cigaretteBrand")
   (ListLink
      (ConceptNode "dunhill")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_cigaretteBrand")
   (ListLink
      (ObjectNode "blend")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_cigaretteBrand")
   (ListLink
      (ConceptNode "bluemaster")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_cigaretteBrand")
   (ListLink
      (ConceptNode "prince")
      (ConceptNode "true")
   )
)


; Drinks
(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "tea")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "coffee")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "milk")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "beer")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_drink")
   (ListLink
      (ConceptNode "water")
      (ConceptNode "true")
   )
)

; House  inheritance definitions
(EvaluationLink (stv 1 1)
   (PredicateNode "is_house")
   (ListLink
      (ObjectNode "firstHouse")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_house")
   (ListLink
      (ObjectNode "secondHouse")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_house")
   (ListLink
      (ObjectNode "thirdHouse")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_house")
   (ListLink
      (ObjectNode "fourthHouse")
      (ConceptNode "true")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "is_house")
   (ListLink
      (ObjectNode "fifthHouse")
      (ConceptNode "true")
   )
)

; House leftOf definitions
(EvaluationLink (stv 1 1)
   (PredicateNode "leftOf")
   (ListLink
      (ObjectNode "firstHouse")
      (ObjectNode "secondHouse")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "leftOf")
   (ListLink
      (ObjectNode "secondHouse")
      (ObjectNode "thirdHouse")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "leftOf")
   (ListLink
      (ObjectNode "thirdHouse")
      (ObjectNode "fourthHouse")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "leftOf")
   (ListLink
      (ObjectNode "fourthHose")
      (ObjectNode "fifthHouse")
   )
)
      
; House rightOf definitions
(EvaluationLink (stv 1 1)
   (PredicateNode "rightOf")
   (ListLink
      (ObjectNode "secondHouse")
      (ObjectNode "firstHouse")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "rightOf")
   (ListLink
      (ObjectNode "thirdHouse")
      (ObjectNode "secondHouse")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "rightOf")
   (ListLink
      (ObjectNode "fourthHouse")
      (ObjectNode "thirdHouse")
   )
)

(EvaluationLink (stv 1 1) 
   (PredicateNode "rightOf")
   (ListLink
      (ObjectNode "fifthHouse")
      (ObjectNode "fourthHouse")
   )
)

(EvaluationLink (stv 1 1) 
   (PredicateNode "livesIn")
   (ListLink
      (AvatarNode "id_man_1")
      (ObjectNode "firstHouse")
   )
)      

(EvaluationLink (stv 1 1) 
   (PredicateNode "livesIn")
   (ListLink
      (AvatarNode "id_man_2")
      (ObjectNode "secondHouse")
   )
)      

(EvaluationLink (stv 1 1) 
   (PredicateNode "livesIn")
   (ListLink
      (AvatarNode "id_man_3")
      (ObjectNode "thirdHouse")
   )
)      

(EvaluationLink (stv 1 1) 
   (PredicateNode "livesIn")
   (ListLink
      (AvatarNode "id_man_4")
      (ObjectNode "fourthHouse")
   )
)      

(EvaluationLink (stv 1 1) 
   (PredicateNode "livesIn")
   (ListLink
      (AvatarNode "id_man_5")
      (ObjectNode "fifthHouse")
   )
)      


