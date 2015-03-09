-- This file contains my first attempt to represent hypergraphs in
-- Haskell and use the Haskell compiler to type check them

---------------------------------------------
-- Attempt using 'data' to represent atoms --
---------------------------------------------

data TV = NullTV
        | SimpleTV Float Float
        | TVAnd TV TV
        | TVOr TV TV
        | TVEvaluation Evaluation
        | TVMember Member
        | TVGetTV Atom

data Atom = AtomPredicate Predicate
          | AtomConcept Concept
          | AtomNumber Float
          | AtomMember Member
          | AtomList List
          | AtomSchema Schema

data Concept = Concept String
             | ConceptAnd Concept Concept
             | ConceptOr Concept Concept

data Member = Member Atom Concept

data Predicate = Predicate (Atom -> TV)
               | PredicateAnd Predicate Predicate
               | PredicateOr Predicate Predicate

data Evaluation = Evaluation Predicate Atom

data Schema = Schema (Atom -> Atom)
data ExecutionOutput = ExecutionOutput Schema Atom

data List = List [Atom]

-------------
-- Example --
-------------

-- Functions from [Atom] to TV to build predicates
is_bottom :: Atom -> TV
is_bottom = undefined
is_top :: Atom -> TV
is_top = undefined
is_car :: Atom -> TV
is_car = undefined

-- And/Or hypergraph of concepts
h1 = ConceptOr (ConceptAnd (Concept "A") (Concept "B")) (Concept "C")

-- And/Or hypergraph of predicates
h2 = PredicateOr (PredicateAnd (Predicate is_top) (Predicate is_car)) (Predicate is_bottom)

-- Apply EvaluationLink to predicate h2
tv3 = TVEvaluation (Evaluation h2 (AtomConcept (Concept "BMW")))

-- Build a SchemaLink
add :: Atom -> Atom
add (AtomList (List [AtomNumber x, AtomNumber y])) = AtomNumber (x + y)
add _ = undefined
h4 = Schema add

-- Test GetTVLink
tv1 = TVGetTV (AtomConcept h1)
tv2 = TVGetTV (AtomPredicate h2)

-- Test AndLink with TV
tv4 = TVAnd tv1 tv3
