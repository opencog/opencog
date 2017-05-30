-- GSoC 2015 - Haskell bindings for OpenCog.

module BackgroundKnowledge (bkn) where

import OpenCog.AtomSpace   (Atom(..),AtomGen,noTv,stv,(|>),(\>),atomList)

bkn :: [AtomGen]
bkn = atomList
 |> ForAllLink (stv 1 1)
        (ListLink |> VariableNode "X"
                  |> VariableNode "Y"
                  \> VariableNode "Z" )
        (ImplicationLink noTv
            (AndLink noTv
               |> EvaluationLink noTv
                    (PredicateNode "take" noTv)
                    (ListLink |> VariableNode "X"
                              \> VariableNode "Y"
                    )
               \> EvaluationLink noTv
                    (PredicateNode "contain" noTv)
                    (ListLink |> VariableNode "Y"
                              \> VariableNode "Z"
                    )
            )
            (EvaluationLink noTv
                (PredicateNode "take" noTv)
                (ListLink |> VariableNode "X"
                          \> VariableNode "Z"
                )
            )
        )
 |> PredicateNode "take-treatment-1" (stv 0.1 0.8)
 |> PredicateNode "take-compound-A" (stv 0.2 0.8)
 |> EquivalenceLink (stv 1 1)
       (PredicateNode "take-treatment-1" noTv)
       (EvaluationLink noTv
          (PredicateNode "take" noTv)
          (ListLink |> VariableNode "X"
                    \> ConceptNode "treatment-1" noTv
          )
       )
 |> EquivalenceLink (stv 1 1)
        (PredicateNode "take-compound-A" noTv)
        (EvaluationLink noTv
            (PredicateNode "take" noTv)
            (ListLink |> VariableNode "$X"
                      \> ConceptNode "compound-A" noTv
            )
        )
 |> EvaluationLink (stv 1 1)
        (PredicateNode "contain" noTv)
        (ListLink |> ConceptNode "treatment-1" noTv
                  \> ConceptNode "compound-A" noTv
        )
 |> ImplicationLink (stv 0.55 0.8)
        (PredicateNode "take-compound-A" noTv)
        (PredicateNode "recovery-speed-of-injury-alpha" noTv)
 |> PredicateNode "take-treatment-2" (stv 0.05 0.8)
 |> EquivalenceLink (stv 1 1)
        (PredicateNode "take-treatment-2" noTv)
        (EvaluationLink noTv
            (PredicateNode "take" noTv)
            (ListLink |> VariableNode "X"
                      \> ConceptNode "treatment-2" noTv
            )
        )
 |> EquivalenceLink (stv 1 1)
        (PredicateNode "take-compound-B" noTv)
        (EvaluationLink noTv
            (PredicateNode "take" noTv)
            (ListLink |> VariableNode "$X"
                      \> ConceptNode "compound-B" noTv
            )
        )
 |> EvaluationLink (stv 0.99 0.99)
        (PredicateNode "contain" noTv)
        (ListLink |> ConceptNode "treatment-2" noTv
                  \> ConceptNode "compound-B" noTv
        )
 |> ImplicationLink (stv 0.8 0.6)
        (PredicateNode "take-compound-B" noTv)
        (PredicateNode "recovery-speed-of-injury-alpha" noTv)
 |> PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.8)
 |> PredicateNode "is-well-hydrated" noTv

 |> ImplicationLink (stv 0.85 0.95)
        (PredicateNode "eat-lots-fruits-vegetables" noTv)
        (PredicateNode "is-well-hydrated" noTv)
 |> AverageLink (stv 0.7 0.6)
        (VariableNode "X")
        (ImplicationLink noTv
            (MemberLink noTv
                (VariableNode "X")
                (ConceptNode "injury-recovery-speed-predicates" noTv)
            )
            (ImplicationLink noTv
                (PredicateNode "is-well-hydrated" noTv)
                (VariableNode "X")
            )
        )
 |> PredicateNode "recovery-speed-of-injury-alpha" (stv 0.02 0.8)
 |> PredicateNode "recovery-speed-of-injury-alpha" noTv
 \> MemberLink (stv 1 1)
        (PredicateNode "recovery-speed-of-injury-alpha" noTv)
        (ConceptNode "injury-recovery-speed-predicates" noTv)

