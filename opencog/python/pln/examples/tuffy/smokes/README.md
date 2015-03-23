PLN version of Tuffy MLN "smokes" sample
========================================

More details on this example are available here:

- https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
- http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

Author: Cosmo Harrigan, March 2014

##### Input data used

###### Python format
https://github.com/opencog/opencog/blob/master/opencog/python/pln/examples/tuffy/smokes/data.py

###### Scheme format
https://github.com/opencog/test-datasets/blob/master/pln/tuffy/smokes/smokes.scm

## Results
```
(EvaluationLink (stv 0.622500 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

(EvaluationLink (stv 0.503926 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

(EvaluationLink (stv 0.452951 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

(EvaluationLink (stv 0.452951 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)
```

## Active rules

```
ModusPonensRule
```

## AtomSpace starting contents

```
(ConceptNode "Bob") ; [22]

(ConceptNode "Frank") ; [27]

(ConceptNode "Anna") ; [6]

(ConceptNode "Edward") ; [10]

(ListLink (stv 1.000000 0.000000)
  (ConceptNode "Edward") ; [10]
) ; [11]

(ListLink (stv 1.000000 0.000000)
  (ConceptNode "Anna") ; [6]
  (ConceptNode "Bob") ; [22]
) ; [23]

(ListLink (stv 1.000000 0.000000)
  (ConceptNode "Bob") ; [22]
  (ConceptNode "Anna") ; [6]
) ; [25]

(ListLink (stv 1.000000 0.000000)
  (ConceptNode "Edward") ; [10]
  (ConceptNode "Frank") ; [27]
) ; [28]

(ListLink (stv 1.000000 0.000000)
  (ConceptNode "Anna") ; [6]
) ; [8]

(PredicateNode "cancer") ; [13]

(PredicateNode "friends") ; [17]

(PredicateNode "smokes") ; [7]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward") ; [10]
  ) ; [11]
) ; [12]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "friends") ; [17]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna") ; [6]
    (ConceptNode "Bob") ; [22]
  ) ; [23]
) ; [24]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (VariableNode "$X") ; [1]
  ) ; [2]
) ; [14]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "friends") ; [17]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob") ; [22]
    (ConceptNode "Anna") ; [6]
  ) ; [25]
) ; [26]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "friends") ; [17]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward") ; [10]
    (ConceptNode "Frank") ; [27]
  ) ; [28]
) ; [29]

(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna") ; [6]
  ) ; [8]
) ; [9]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "friends") ; [17]
  (ListLink (stv 1.000000 0.000000)
    (VariableNode "$X") ; [1]
    (VariableNode "$Y") ; [3]
  ) ; [5]
) ; [18]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (VariableNode "$Y") ; [3]
  ) ; [4]
) ; [19]

(EvaluationLink (stv 1.000000 0.000000)
  (PredicateNode "cancer") ; [13]
  (ListLink (stv 1.000000 0.000000)
    (VariableNode "$X") ; [1]
  ) ; [2]
) ; [15]

(VariableNode "$X") ; [1]

(VariableNode "$Y") ; [3]

(ListLink (stv 1.000000 0.000000)
  (VariableNode "$X") ; [1]
) ; [2]

(ListLink (stv 1.000000 0.000000)
  (VariableNode "$Y") ; [3]
) ; [4]

(ListLink (stv 1.000000 0.000000)
  (VariableNode "$X") ; [1]
  (VariableNode "$Y") ; [3]
) ; [5]


(ImplicationLink (stv 0.622500 1.000000)
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (VariableNode "$X") ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "cancer") ; [13]
    (ListLink (stv 1.000000 0.000000)
      (VariableNode "$X") ; [1]
    ) ; [2]
  ) ; [15]
) ; [16]

(ImplicationLink (stv 1.000000 0.000000)
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (VariableNode "$X") ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (VariableNode "$Y") ; [3]
    ) ; [4]
  ) ; [19]
) ; [20]

(ImplicationLink (stv 0.598700 1.000000)
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "friends") ; [17]
    (ListLink (stv 1.000000 0.000000)
      (VariableNode "$X") ; [1]
      (VariableNode "$Y") ; [3]
    ) ; [5]
  ) ; [18]
  (ImplicationLink (stv 1.000000 0.000000)
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "smokes") ; [7]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$X") ; [1]
      ) ; [2]
    ) ; [14]
    (EvaluationLink (stv 1.000000 0.000000)
      (PredicateNode "smokes") ; [7]
      (ListLink (stv 1.000000 0.000000)
        (VariableNode "$Y") ; [3]
      ) ; [4]
    ) ; [19]
  ) ; [20]
) ; [21]
```

## Inference steps

```
----- [Output # 1] -----
-- Output:
(ImplicationLink (stv 0.598700 1.000000)
  (EvaluationLink (stv 1.000000 1.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Anna") ; [6]
    ) ; [8]
  ) ; [9]
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Bob") ; [22]
    ) ; [36]
  ) ; [37]
) ; [38]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
    ) ; [5]
  ) ; [18]
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      ) ; [2]
    ) ; [14]
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
      ) ; [4]
    ) ; [19]
  ) ; [20]
) ; [21]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
  ) ; [23]
) ; [24]
]

----- [Output # 2] -----
-- Output:
(EvaluationLink (stv 0.598700 1.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob") ; [22]
  ) ; [36]
) ; [37]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
    ) ; [8]
  ) ; [9]
  (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
    ) ; [36]
  ) ; [37]
) ; [38]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
  ) ; [8]
) ; [9]
]

----- [Output # 3] -----
-- Output:
(EvaluationLink (stv 0.622500 1.000000)
  (PredicateNode "cancer") ; [13]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward") ; [10]
  ) ; [11]
) ; [54]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.622500 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000)) ; [13]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [15]
) ; [16]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000)) ; [10]
  ) ; [11]
) ; [12]
]

----- [Output # 4] -----
-- Output:
(ImplicationLink (stv 0.598700 1.000000)
  (EvaluationLink (stv 1.000000 1.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Edward") ; [10]
    ) ; [11]
  ) ; [12]
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Frank") ; [27]
    ) ; [62]
  ) ; [63]
) ; [64]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
    ) ; [5]
  ) ; [18]
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      ) ; [2]
    ) ; [14]
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
      ) ; [4]
    ) ; [19]
  ) ; [20]
) ; [21]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000)) ; [10]
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000)) ; [27]
  ) ; [28]
) ; [29]
]

----- [Output # 5] -----
-- Output:
(EvaluationLink (stv 0.598700 1.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank") ; [27]
  ) ; [62]
) ; [63]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000)) ; [10]
    ) ; [11]
  ) ; [12]
  (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000)) ; [27]
    ) ; [62]
  ) ; [63]
) ; [64]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000)) ; [10]
  ) ; [11]
) ; [12]
]

----- [Output # 6] -----
-- Output:
(EvaluationLink (stv 0.452951 1.000000)
  (PredicateNode "cancer") ; [13]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob") ; [22]
  ) ; [36]
) ; [79]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.622500 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000)) ; [13]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [15]
) ; [16]
, (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
  ) ; [36]
) ; [37]
]

----- [Output # 7] -----
-- Output:
(EvaluationLink (stv 0.452951 1.000000)
  (PredicateNode "cancer") ; [13]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank") ; [27]
  ) ; [62]
) ; [90]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.622500 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000)) ; [13]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [15]
) ; [16]
, (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000)) ; [27]
  ) ; [62]
) ; [63]
]

----- [Output # 8] -----
-- Output:
(ImplicationLink (stv 0.598700 1.000000)
  (EvaluationLink (stv 0.598700 1.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Bob") ; [22]
    ) ; [36]
  ) ; [37]
  (EvaluationLink (stv 1.000000 1.000000)
    (PredicateNode "smokes") ; [7]
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Anna") ; [6]
    ) ; [8]
  ) ; [9]
) ; [134]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
    ) ; [5]
  ) ; [18]
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
      ) ; [2]
    ) ; [14]
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
      ) ; [4]
    ) ; [19]
  ) ; [20]
) ; [21]
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000)) ; [17]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
  ) ; [25]
) ; [26]
]

----- [Output # 9] -----
-- Output:
(EvaluationLink (stv 0.719351 1.000000)
  (PredicateNode "smokes") ; [7]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna") ; [6]
  ) ; [8]
) ; [9]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.598700 1.000000)
  (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
    ) ; [36]
  ) ; [37]
  (EvaluationLink (av 0 0 0) (stv 0.719351 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
    ) ; [8]
  ) ; [9]
) ; [134]
, (EvaluationLink (av 0 0 0) (stv 0.598700 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000)) ; [22]
  ) ; [36]
) ; [37]
]

----- [Output # 10] -----
-- Output:
(EvaluationLink (stv 0.503926 1.000000)
  (PredicateNode "cancer") ; [13]
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna") ; [6]
  ) ; [8]
) ; [164]

-- using production rule: ModusPonensRule<ImplicationLink>

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.622500 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [14]
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000)) ; [13]
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000)) ; [1]
    ) ; [2]
  ) ; [15]
) ; [16]
, (EvaluationLink (av 0 0 0) (stv 0.719351 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000)) ; [7]
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000)) ; [6]
  ) ; [8]
) ; [9]
]

---- Answer found after 10 inference steps that produced a new output, out of 28 total inference steps attempted.
```
