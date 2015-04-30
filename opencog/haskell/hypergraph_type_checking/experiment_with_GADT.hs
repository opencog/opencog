-- Attempt to using Haskell compiler to type check hypergraphs, using
-- an extension of data call Generalized Algebraic Data Type

{-#LANGUAGE GADTs #-}

data TV = SimpleTV Float Float

data ConceptName = ConceptName String

data VariableName = VariableName String

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
    Implication :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom TV
    Equivalence :: (a ~ (Atom b -> TV)) => Atom a -> Atom a -> Atom TV
    Evaluation :: (Atom (Atom a -> TV)) -> Atom a -> Atom TV

    -- Concept
    Concept :: ConceptName -> Atom ConceptName
    ConceptAnd :: Atom ConceptName -> Atom ConceptName -> Atom ConceptName
    ConceptOr :: Atom ConceptName -> Atom ConceptName -> Atom ConceptName
    Inheritance :: Atom ConceptName -> Atom ConceptName -> Atom TV
    Similarity :: Atom ConceptName -> Atom ConceptName -> Atom TV
    Member :: Atom a -> Atom ConceptName -> Atom TV
    SatisfyingSet :: Atom (Atom a -> TV) -> Atom ConceptName

    -- Variable
    PredicateVariable :: VariableName -> Atom (Atom a -> TV)
    ConceptVariable :: VariableName -> Atom ConceptName
    TVVariable :: VariableName -> Atom TV
    Variable :: VariableName -> Atom a

    -- Binding
    Bind :: Atom a -> Atom TV -> Atom (Atom a -> TV)

    -- Quantifier
    ForAll :: Atom a -> Atom TV -> Atom TV
    Exists :: Atom a -> Atom TV -> Atom TV

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
a = Concept (ConceptName "A")
b = Concept (ConceptName "B")
c = Concept (ConceptName "C")
h1 = ConceptOr (ConceptAnd a b) c

-- And/Or hypergraph of predicates
h2 = PredicateOr (PredicateAnd (Predicate is_top) (Predicate is_car)) (Predicate is_bottom)

-- Apply Evaluation to predicate h2
tv3 = Evaluation h2 (List [Concept (ConceptName "BMW")])

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

-- And/Or hypergraph of concepts involving a variable
x = Variable (VariableName "X")
y = Variable (VariableName "Y")
h6 = ConceptOr (ConceptAnd a x) y

-- Define a predicate with Bind
h7 = Bind (List [x, y]) (TVGetTV h6)

-- Quantifier
h8 = ForAll (List [x, y]) (TVGetTV h6)

-- More complex free variable body. In the following the type checker
-- has no problem with that, this is wrong because x is actually a
-- concept variable being used in a TVOr, it cannot be Atom TV and
-- Atom ConceptName at the same time
h9 = TVOr (TVGetTV h6) x

-- In order to work around that one can use explicitely
-- the type constructor ConceptVariable
xc = ConceptVariable (VariableName "X")
yc = ConceptVariable (VariableName "Y")
h6b = ConceptOr (ConceptAnd a xc) yc
-- h9b = TVOr (TVGetTV h6) xc -- The type checker raises an error

----------------
-- Dummy main --
----------------

main :: IO ()
main = undefined
