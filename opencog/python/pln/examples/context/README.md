# Contextual inference example





## Some use cases by Ben

```
(define universe (ConceptNode "KnownPhysicalUniverse" (stv .001 .99))
(define living (ConceptNode "Living" (stv .001 .99))
(define earth (ConceptNode "Earth" (stv .001 .99))

(ContextLink (stv .00001 .99)
    universe
    living
)

(ContextLink (stv .1 .99)
    earth
    living
)

(define cogworld (ConceptNode "CogWorld" (stv .001 .99))
(define composed_of (PredicateNode "composed_of" (stv .001 .99))
(define tree (ConceptNode "Tree" (stv .001 .99))
(define block (ConceptNode "Block" (stv .001 .99))
(define reality (ConceptNode "HumanConsensusReality (stv .001 .99))

(ContextLink 
    cogworld 
    (EvaluationLink (stv .001 .99) 
        composed_of 
        (ListLink 
            tree 
            block
            )
        )
    )
)
             

(ContextLink 
    reality
    EvaluationLink <.0,.9999> 
            PredicateNode: composed_of 
            ListLink 
                tree 
                block 
                   
EvaluationLink <.001,.99> 
   PredicateNode: composed_of 
      ListLink 
        tree 
        block 
                   
                   
ContextLink 
     ConceptNode: CogWorld 
      EvaluationLink <.9,.99> 
            PredicateNode: has_part 
            ListLink 
                   tree 
                   leaf 

but 



ContextLink 
      ConceptNode: HumanConsensusReality 
      EvaluationLink <.9,.9999> 
            PredicateNode: has_part 
            ListLink 
                   tree 
                   leaf 
                   
```