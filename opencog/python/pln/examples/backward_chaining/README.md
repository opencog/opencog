# Backward-chaining demo

![Proof tree](backward_chaining_proof_tree.jpg)
The proof tree that should be constructed by backward-chaining.

## Inference path

```
(ImplicationLink
    (AndLink
        (InheritanceLink
            (VariableNode "$x")
            (ConceptNode "American"))
        (InheritanceLink
            (VariableNode "$y")
            (ConceptNode "weapon"))
        (EvaluationLink
            (PredicateNode "sell")
            (ListLink
                (VariableNode "$x")
                (VariableNode "$y")
                (VariableNode "$z")))
        (InheritanceLink
            (VariableNode "$z")
            (ConceptNode "hostile")))
    (InheritanceLink
        (VariableNode "$x")
        (ConceptNode "criminal")))
```

### First condition
"$x" gets bound to "West" because 
(InheritanceLink
    (ConceptNode "West")
    (ConceptNode "American"))
    
### Second condition
"$y" should be bound to "missile@123" --> how?

### Third condition
"$z" should be bound to "Nono"

ImplicationLink
    (AndLink
        (InheritanceLink
            (VariableNode "$a")
            (ConceptNode "missile"))
        (EvaluationLink
            (PredicateNode "own")
            (ListLink
                (ConceptNode "Nono")
                (VariableNode "$a"))))
    (EvaluationLink
        (PredicateNode "sell")
        (ListLink
            (ConceptNode "West")
            (VariableNode "$a")
            (ConceptNode "Nono"))))