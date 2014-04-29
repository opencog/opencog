(ConceptNode "dog" (stv 1 1))
(ConceptNode "cat" (stv 1 1))
(ConceptNode "human" (stv 1 1))
(ConceptNode "animal" (stv 1 1))
(ConceptNode "meat" (stv 1 1))

(ConceptNode "American" (stv 1 1))
(ConceptNode "Chinese" (stv 1 1))
(ConceptNode "European" (stv 1 1))

(PredicateNode "eat" (stv 1 1))
(PredicateNode "drink" (stv 1 1))

(ObjectNode "KittyPie" (stv 1 1))
(ObjectNode "DoggiPie" (stv 1 1))
(ObjectNode "Bob" (stv 1 1))
(ObjectNode "Bill" (stv 1 1))
(ObjectNode "LiMing" (stv 1 1))
(ObjectNode "WuNing" (stv 1 1))
(ObjectNode "Andrew" (stv 1 1))
(ObjectNode "Hugo" (stv 1 1))

(ConceptNode "Coke")
(ConceptNode "tea")
(ConceptNode "milk")


(InheritanceLink (stv 1 1)
   (ObjectNode "DoggiPie")
   (ConceptNode "dog")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "KittyPie")
   (ConceptNode "cat")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Bob")
   (ConceptNode "human")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "WuNing")
   (ConceptNode "human")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "LiMing")
   (ConceptNode "Chinese")
)

(InheritanceLink (stv 1 1)
   (ConceptNode "human")
   (ConceptNode "animal")
)

(InheritanceLink (stv 1 1)
   (ConceptNode "cat")
   (ConceptNode "animal")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "WuNing")
   (ConceptNode "Chinese")
)


(InheritanceLink (stv 1 1)
   (ConceptNode "dog")
   (ConceptNode "animal")
)

(EvaluationLink (stv 1 1)
   (PredicateNode "eat")
   (ListLink
      (ObjectNode "KittyPie")
      (ConceptNode "meat")
   )
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Bob")
   (ConceptNode "American")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Bill")
   (ConceptNode "American")
)

(EvaluationLink (stv 1 1)
   (PredicateNode "eat")
   (ListLink
      (ObjectNode "Bob")
      (ConceptNode "meat")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "eat")
   (ListLink
      (ObjectNode "Bill")
      (ConceptNode "bread")
   )
)
       
(EvaluationLink (stv 1 1)
   (PredicateNode "eat")
   (ListLink
      (ObjectNode "DoggiPie")
      (ConceptNode "meat")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "drink")
   (ListLink
      (ObjectNode "Bob")
      (ConceptNode "Coke")
   )
)

(EvaluationLink (stv 1 1)
   (PredicateNode "drink")
   (ListLink
      (ObjectNode "LiMing")
      (ConceptNode "tea")
   )
)
   
(EvaluationLink (stv 1 1)
   (PredicateNode "drink")
   (ListLink
      (ObjectNode "Andrew")
      (ConceptNode "milk")
   )
)   

(InheritanceLink (stv 1 1)
   (ObjectNode "LiMing")
   (ConceptNode "human")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Andrew")
   (ConceptNode "human")
)

(EvaluationLink (stv 1 1)
   (PredicateNode "drink")
   (ListLink
      (ObjectNode "Bill")
      (ConceptNode "Coke")
   )
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Hugo")
   (ConceptNode "human")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Andrew")
   (ConceptNode "European")
)

(InheritanceLink (stv 1 1)
   (ObjectNode "Hugo")
   (ConceptNode "European")
)

(EvaluationLink (stv 1 1)
   (PredicateNode "drink")
   (ListLink
      (ObjectNode "Hugo")
      (ConceptNode "tea")
   )
) 

