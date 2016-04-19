-- GSoC 2015 - Haskell bindings for OpenCog.
{-# LANGUAGE DataKinds #-}

module MosesModel (mosesModel) where

import OpenCog.AtomSpace   (Atom(..),AtomType(ImplicationT),stv,noTv,(|>),(\>))

mosesModel :: Atom ImplicationT
mosesModel = ImplicationLink (stv 0.875 0.44)
                (OrLink noTv
                    |> PredicateNode "take-treatment-1" noTv
                    \> PredicateNode "eat-lots-fruits-vegetables" noTv
                )
                (PredicateNode "recovery-speed-of-injury-alpha" noTv)

