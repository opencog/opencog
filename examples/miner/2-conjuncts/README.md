Simple 2-conjunction pattern miner example, given
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
