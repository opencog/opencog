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
    PredicateOr :: (Atom (Atom a -> TV)) -> (Atom (Atom a -> TV))
                -> (Atom (Atom a -> TV))
    ConceptOr :: (Atom String) -> (Atom String)
              -> (Atom String)
    TVOr :: (Atom TV) -> (Atom TV)
          -> (Atom TV)
    Evaluation :: (Atom (Atom a -> TV)) -> (Atom a) -> Atom TV
    ExecutionOutput :: (Atom (Atom a -> TV)) -> (Atom a) -> (Atom a)
    Schema :: (Atom a -> Atom a) -> (Atom (Atom a -> Atom a))
    List :: [Atom a] -> (Atom [Atom a])

-------------
-- Example --
-------------

-- Functions from Atom to TV to build predicates
is_bottom :: Atom a -> TV
is_bottom = undefined
is_top :: Atom a -> TV
is_top = undefined
is_car :: Atom a -> TV
is_car = undefined

-- And/Or hypergraph of concepts
h1 = ConceptOr (ConceptAnd (Concept "A") (Concept "B")) (Concept "C")

-- And/Or hypergraph of predicates
h2 = PredicateOr (PredicateAnd (Predicate is_top) (Predicate is_car)) (Predicate is_bottom)

-- Apply EvaluationLink to predicate h2
tv3 = Evaluation h2 (List [Concept "BMW"])

-- Build a SchemaLink
add :: Atom a -> Atom a
add (List [Number x, Number y]) = Number (x + y)
add _ = undefined
h4 = Schema add

-- Test GetTVLink
tv1 = GetTV h1
tv2 = GetTV h2

-- Test AndLink with TV
tv4 = TVAnd tv1 tv3
