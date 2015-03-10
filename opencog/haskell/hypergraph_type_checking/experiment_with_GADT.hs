-- Attempt to using Haskell compiler to type check hypergraphs, using
-- an extension of data call Generalized Algebraic Data Type

{-#LANGUAGE GADTs #-}

data TV -- TODO = (Weight Float, Confidence Float)

data Atom a where
    -- TV
    TV :: TV -> Atom TV
    TVAnd :: Atom TV -> Atom TV -> Atom TV
    TVOr :: Atom TV -> Atom TV -> Atom TV
    TVGetTV :: Atom a -> Atom TV

    -- Predicate
    Predicate :: (Atom a -> TV) -> Atom (Atom a -> TV)
    PredicateAnd :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom a
    PredicateOr :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom a
    Evaluation :: (Atom (Atom a -> TV)) -> Atom a -> Atom TV
    Implication :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom TV
    Equivalence :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom TV

    -- Concept
    Concept :: String -> Atom String
    ConceptAnd :: Atom String -> Atom String -> Atom String
    ConceptOr :: Atom String -> Atom String -> Atom String
    Member :: Atom a -> Atom String -> Atom TV
    Inheritance :: Atom String -> Atom String -> Atom TV
    Similarity :: Atom String -> Atom String -> Atom TV
    SatisfyingSet :: Atom (Atom a -> TV) -> Atom String

    -- -- Variable
    -- Variable :: Atom 
    
    -- Bind :: List Atom (Atom a -> TV)

    -- number
    Number :: (Num a) => a -> Atom a

    -- Schema
    Schema :: (a ~ (Atom b -> Atom c)) => a -> Atom a
    ExecutionOutput :: (a ~ (Atom b -> Atom c)) => Atom a -> Atom b -> Atom c

    -- List
    List :: [Atom a] -> Atom [Atom a]

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

-- Apply Evaluation to predicate h2
tv3 = Evaluation h2 (List [Concept "BMW"])

-- Build a Schema
add :: (Atom [Atom Float]) -> Atom Float
add (List [Number x, Number y]) = Number (x + y)
add _ = undefined
h4 = Schema add -- :: Atom (Atom [Atom Float] -> Atom Float)

-- Apply a Schema
h5 = ExecutionOutput h4 (List [Number 3, Number 4]) -- h5 :: Atom Float

-- Test GetTVLink
tv1 = TVGetTV h1
tv2 = TVGetTV h2

-- Test AndLink with TV
tv4 = TVAnd tv1 tv3

----------------
-- Dummy main --
----------------

main :: IO ()
main = undefined
