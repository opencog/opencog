-- Attempt at using Haskell compiler to type check hypergraphs, using
-- an extension of data call Generalized Algebraic Data Type

{-#LANGUAGE GADTs #-}

data TV -- TODO = (Weight Float, Confidence Float)

data Atom a where
    TV :: TV -> (Atom TV)
    GetTV :: (Atom a) -> (Atom TV)
    Predicate :: (Atom a -> TV) -> (Atom (Atom a -> TV))
    Concept :: String -> Atom String
    Number :: (Num a) => a -> Atom a
    PredicateAnd :: (Atom (Atom a -> TV)) -> (Atom (Atom a -> TV))
                 -> (Atom (Atom a -> TV))
    ConceptAnd :: (Atom String) -> (Atom String)
               -> (Atom String)
    TVAnd :: (Atom TV) -> (Atom TV)
          -> (Atom TV)
    Evaluation :: (Atom a) -> (Atom (Atom a -> TV)) -> Atom TV
    ExecutionOutput :: (Atom a) -> (Atom (Atom a -> TV))
                    -> (Atom (Atom a -> TV))
    List :: [Atom a] -> (Atom [Atom a])

-------------
-- Example --
-------------

-- Functions from Atom to TV to build predicates
is_bottom :: Atom a -> TV
is_bottom _ = undefined
-- is_top :: Atom a -> TV
-- is_top _ = TV 1 0.9
-- is_car :: Atom a -> TV
-- is_car (Concept "BMW") = TV 1 0.9
-- is_car (Concept "Camel") = TV 0.1 0.9
-- is_car _ = TV 0 0.9

