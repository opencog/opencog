Ultra simple example, given
```
(Inheritance
  (Concept "A")
  (Concept "B"))
(Inheritance
  (Concept "A")
  (Concept "C"))
```
find pattern
```
(Lambda
  (Variable "$X")
  (Inheritance
    (Concept "A")
    (Variable "$X")))
```
