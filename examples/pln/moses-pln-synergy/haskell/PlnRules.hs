-- GSoC 2015 - Haskell bindings for OpenCog.
{-# LANGUAGE DataKinds #-}

module PlnRules (
    plnRuleEquivalenceHack
  , plnRuleEliminateNeutralElementHack
  , plnRuleEliminateDanglingJunctorHack
  , plnRuleAndHack
  , plnRuleForAllHack
  , plnRuleAverageHack
  ) where

import OpenCog.AtomSpace   (Atom(..),AtomType(BindT),stv,noTv,(|>),(\>))

plnRuleEquivalenceHack :: Atom BindT
plnRuleEquivalenceHack = BindLink
     (VariableList [])
     (ImplicationLink noTv
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> QuoteLink (VariableNode "X")
              \> PredicateNode "treatment-1" noTv
           )
        )
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> QuoteLink (VariableNode "X")
              \> PredicateNode "compound-A" noTv
           )
        )
     )
     (ImplicationLink (stv 1 1)
         (PredicateNode "take-treatment-1" noTv)
         (PredicateNode "take-compound-A" noTv)
     )

plnRuleEliminateNeutralElementHack :: Atom BindT
plnRuleEliminateNeutralElementHack = BindLink
     (VariableList [])
     (ImplicationLink noTv
        (AndLink noTv
           |> EvaluationLink noTv
                (PredicateNode "take" noTv)
                (ListLink
                   |> QuoteLink (VariableNode "X")
                   \> PredicateNode "treatment-1" noTv
                )
           \> EvaluationLink noTv
                (PredicateNode "contain" noTv)
                (ListLink
                   |> PredicateNode "treatment-1" noTv
                   \> PredicateNode "compound-A" noTv
                )
        )
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> QuoteLink (VariableNode "X")
              \> PredicateNode "compound-A" noTv
           )
        )
     )
     (ImplicationLink (stv 1 1)
        (AndLink noTv
           \> EvaluationLink noTv
                (PredicateNode "take" noTv)
                (ListLink
                   |> VariableNode "X"
                   \> PredicateNode "treatment-1" noTv
                )
        )
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> VariableNode "X"
              \> PredicateNode "compound-A" noTv
           )
        )
     )

plnRuleEliminateDanglingJunctorHack :: Atom BindT
plnRuleEliminateDanglingJunctorHack = BindLink
     (VariableList [])
     (ImplicationLink noTv
        (AndLink noTv
           \> EvaluationLink noTv
                (PredicateNode "take" noTv)
                (ListLink
                   |> QuoteLink (VariableNode "X")
                   \> PredicateNode "treatment-1" noTv
                )
        )
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> QuoteLink (VariableNode "X")
              \> PredicateNode "compound-A" noTv
           )
        )
     )
     (ImplicationLink (stv 1 1)
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> VariableNode "X"
              \> PredicateNode "treatment-1" noTv
           )
        )
        (EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> VariableNode "X"
              \> PredicateNode "compound-A" noTv
           )
        )
     )

plnRuleAndHack :: Atom BindT
plnRuleAndHack = BindLink
   (VariableList [])
   (AndLink noTv
      |> EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> QuoteLink (VariableNode "X")
              \> VariableNode "Y"
           )
      \> EvaluationLink noTv
           (PredicateNode "contain" noTv)
           (ListLink
              |> VariableNode "Y"
              \> VariableNode "Z"
           )
   )
   (AndLink (stv 1 1)
      |> EvaluationLink noTv
           (PredicateNode "take" noTv)
           (ListLink
              |> VariableNode "X"
              \> VariableNode "Y"
           )
      \> EvaluationLink noTv
           (PredicateNode "contain" noTv)
           (ListLink
              |> VariableNode "Y"
              \> VariableNode "Z"
           )
   )

plnRuleForAllHack :: Atom BindT
plnRuleForAllHack = BindLink
        (VariableList [])
        (ForAllLink noTv
            (ListLink
                |> QuoteLink (VariableNode "X")
                |> QuoteLink (VariableNode "Y")
                \> QuoteLink (VariableNode "Z")
            )
            (ImplicationLink noTv
                (AndLink noTv
                   |> EvaluationLink noTv
                        (PredicateNode "take" noTv)
                        (ListLink
                            |> QuoteLink (VariableNode "X")
                            \> QuoteLink (VariableNode "Y")
                        )
                   \> EvaluationLink noTv
                        (PredicateNode "contain" noTv)
                        (ListLink
                            |> QuoteLink (VariableNode "Y")
                            \> QuoteLink (VariableNode "Z")
                        )
                )
                (EvaluationLink noTv
                    (PredicateNode "take" noTv)
                    (ListLink
                        |> QuoteLink (VariableNode "X")
                        \> QuoteLink (VariableNode "Z")
                    )
                )
            )
        )
        (ImplicationLink (stv 1 1)
            (AndLink noTv
               |> EvaluationLink noTv
                    (PredicateNode "take" noTv)
                    (ListLink
                        |> VariableNode "X"
                        \> PredicateNode "treatment-1" noTv
                    )
               \> EvaluationLink noTv
                    (PredicateNode "contain" noTv)
                    (ListLink
                        |> PredicateNode "treatment-1" noTv
                        \> PredicateNode "compound-A" noTv
                    )
            )
            (EvaluationLink noTv
                (PredicateNode "take" noTv)
                (ListLink
                    |> VariableNode "X"
                    \> PredicateNode "compound-A" noTv
                )
            )
        )

plnRuleAverageHack :: Atom BindT
plnRuleAverageHack = BindLink
     (VariableList [])
     (AverageLink noTv
        (QuoteLink (VariableNode "X"))
        (ImplicationLink noTv
           (MemberLink noTv
              (QuoteLink (VariableNode "X"))
              (ConceptNode "injury-recovery-speed-predicates" noTv)
           )
           (ImplicationLink noTv
              (PredicateNode "is-well-hydrated" noTv)
              (QuoteLink (VariableNode "X"))
           )
        )
     )
     (ImplicationLink (stv 0.7 0.6)
        (MemberLink noTv
           (PredicateNode "recovery-speed-of-injury-alpha" noTv)
           (ConceptNode "injury-recovery-speed-predicates" noTv)
        )
        (ImplicationLink noTv
           (PredicateNode "is-well-hydrated" noTv)
           (PredicateNode "recovery-speed-of-injury-alpha" noTv)
        )
     )
