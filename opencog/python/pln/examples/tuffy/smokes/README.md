PLN version of Tuffy MLN "smokes" sample
========================================

More details on this example are available here:

- https://github.com/cosmoharrigan/tuffy/tree/master/samples/smoke
- http://hazy.cs.wisc.edu/hazy/tuffy/doc/tuffy-manual.pdf

Author: Cosmo Harrigan, March 2014

**In-progress: incomplete results.**

**The input data used in this file is currently only a subset of the full input data.**

##### Input data used
https://github.com/opencog/opencog/blob/master/opencog/python/pln/examples/tuffy/smokes/data.py

##### Full input data to be used when completed
https://github.com/opencog/test-datasets/blob/master/pln/tuffy/smokes/smokes.scm

## Results
```
(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)
```

## Active rules

```
EvaluationToMemberRule
MemberToEvaluationRule
MemberToInheritanceRule
InheritanceToMemberRule
GeneralEvaluationToMemberRule
ModusPonensRule
```

## AtomSpace starting contents

```
(ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))

(ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))

(ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))

(ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
  (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
)

(ListLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
)

(VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))

(VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))

(ImplicationLink (av 0 0 0) (stv 0.500000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)

(ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)

(ImplicationLink (av 0 0 0) (stv 0.400000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
  (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)

(PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))

(PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))

(PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
```

## Inference steps

```
----- [Output # 1] -----
-- Output:
(ImplicationLink (stv 0.400000 1.000000)
  (EvaluationLink (stv 1.000000 1.000000)
    (PredicateNode "smokes")
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Edward")
    )
  )
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes")
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Frank")
    )
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.400000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 2] -----
-- Output:
(EvaluationLink (stv 0.400000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.400000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 3] -----
-- Output:
(ImplicationLink (stv 0.400000 1.000000)
  (EvaluationLink (stv 1.000000 1.000000)
    (PredicateNode "smokes")
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Anna")
    )
  )
  (EvaluationLink (stv 1.000000 0.000000)
    (PredicateNode "smokes")
    (ListLink (stv 1.000000 0.000000)
      (ConceptNode "Bob")
    )
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.400000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (ImplicationLink (av 0 0 0) (stv 1.000000 0.000000)
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
    (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
      (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
      (ListLink (av 0 0 0) (stv 1.000000 0.000000)
        (VariableNode "$Y" (av 0 0 0) (stv 1.000000 0.000000))
      )
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 4] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "smokes")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 5] -----
-- Output:
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "smokes")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 6] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "smokes")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 7] -----
-- Output:
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 8] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)")
)

-- using production rule: GeneralEvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 9] -----
-- Output:
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 10] -----
-- Output:
(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.500000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 11] -----
-- Output:
(MemberLink (stv 0.400000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "smokes")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 12] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)")
)

-- using production rule: GeneralEvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "friends" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 13] -----
-- Output:
(MemberLink (stv 0.320000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "cancer")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.320000 1.000000)
  (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 14] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 15] -----
-- Output:
(InheritanceLink (stv 0.400000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "smokes")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 16] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "smokes")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 17] -----
-- Output:
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "SatisfyingSet(friends _ Frank:ConceptNode)")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 18] -----
-- Output:
(MemberLink (stv 0.400000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "smokes")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 19] -----
-- Output:
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "smokes")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 20] -----
-- Output:
(EvaluationLink (stv 0.400000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 21] -----
-- Output:
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "SatisfyingSet(friends _ Bob:ConceptNode)")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 22] -----
-- Output:
(InheritanceLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 23] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "SatisfyingSet(friends _ Bob:ConceptNode)")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "SatisfyingSet(friends _ Bob:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 24] -----
-- Output:
(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Frank")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 25] -----
-- Output:
(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.500000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 26] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 27] -----
-- Output:
(EvaluationLink (stv 1.000000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 28] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "SatisfyingSet(friends _ Frank:ConceptNode)")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "SatisfyingSet(friends _ Frank:ConceptNode)" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 29] -----
-- Output:
(EvaluationLink (stv 0.400000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.400000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 30] -----
-- Output:
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "smokes")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 1.000000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 31] -----
-- Output:
(MemberLink (stv 0.400000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "smokes")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 32] -----
-- Output:
(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.500000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 33] -----
-- Output:
(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

-- using production rule: ModusPonensRule

-- based on this input:
[(ImplicationLink (av 0 0 0) (stv 0.500000 1.000000)
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
  (EvaluationLink (av 0 0 0) (stv 1.000000 0.000000)
    (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
    (ListLink (av 0 0 0) (stv 1.000000 0.000000)
      (VariableNode "$X" (av 0 0 0) (stv 1.000000 0.000000))
    )
  )
)
, (EvaluationLink (av 0 0 0) (stv 0.400000 1.000000)
  (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 34] -----
-- Output:
(MemberLink (stv 0.320000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "cancer")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.320000 1.000000)
  (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 35] -----
-- Output:
(InheritanceLink (stv 0.320000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "cancer")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 36] -----
-- Output:
(EvaluationLink (stv 0.320000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 37] -----
-- Output:
(MemberLink (stv 0.500000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "cancer")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.500000 1.000000)
  (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 38] -----
-- Output:
(InheritanceLink (stv 0.500000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "cancer")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 39] -----
-- Output:
(InheritanceLink (stv 0.400000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "smokes")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 40] -----
-- Output:
(MemberLink (stv 0.320000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "cancer")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 41] -----
-- Output:
(MemberLink (stv 0.500000 1.000000)
  (ConceptNode "Edward")
  (ConceptNode "cancer")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 42] -----
-- Output:
(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Edward")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 43] -----
-- Output:
(MemberLink (stv 0.400000 1.000000)
  (ConceptNode "Bob")
  (ConceptNode "smokes")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 44] -----
-- Output:
(MemberLink (stv 0.500000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "cancer")
)

-- using production rule: EvaluationToMemberRule

-- based on this input:
[(EvaluationLink (av 0 0 0) (stv 0.500000 1.000000)
  (PredicateNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
  (ListLink (av 0 0 0) (stv 1.000000 0.000000)
    (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  )
)
]

----- [Output # 45] -----
-- Output:
(EvaluationLink (stv 0.500000 1.000000)
  (PredicateNode "cancer")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Anna")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 46] -----
-- Output:
(InheritanceLink (stv 0.500000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "cancer")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 47] -----
-- Output:
(EvaluationLink (stv 0.400000 1.000000)
  (PredicateNode "smokes")
  (ListLink (stv 1.000000 0.000000)
    (ConceptNode "Bob")
  )
)

-- using production rule: MemberToEvaluationRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.400000 1.000000)
  (ConceptNode "Bob" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 48] -----
-- Output:
(InheritanceLink (stv 0.320000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "cancer")
)

-- using production rule: MemberToInheritanceRule

-- based on this input:
[(MemberLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 49] -----
-- Output:
(MemberLink (stv 0.500000 1.000000)
  (ConceptNode "Anna")
  (ConceptNode "cancer")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.500000 1.000000)
  (ConceptNode "Anna" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]

----- [Output # 50] -----
-- Output:
(MemberLink (stv 0.320000 1.000000)
  (ConceptNode "Frank")
  (ConceptNode "cancer")
)

-- using production rule: InheritanceToMemberRule

-- based on this input:
[(InheritanceLink (av 0 0 0) (stv 0.320000 1.000000)
  (ConceptNode "Frank" (av 0 0 0) (stv 1.000000 0.000000))
  (ConceptNode "cancer" (av 0 0 0) (stv 1.000000 0.000000))
)
]
```
