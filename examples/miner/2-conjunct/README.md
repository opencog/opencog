Simple pattern miner example starting with a pattern with 2
conjunctions. Given the KB
```
(Inheritance
  (Concept "A")
  (Concept "B")))
(Inheritance
  (Concept "B")
  (Concept "C")))
(Inheritance
  (Concept "D")
  (Concept "E")))
(Inheritance
  (Concept "E")
  (Concept "F")))
```
find pattern
```
(Lambda
  (VariableList
    (Variable "$X")
    (Variable "$Y")
    (Variable "$Z"))
  (And
    (Inheritance
      (Variable "$X")
      (Variable "$Y"))
    (Inheritance
      (Variable "$Y")
      (Variable "$Z"))))
```
