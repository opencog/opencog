-- This file contains my first attempt to represent hypergraphs in
-- Haskell and use the Haskell compiler to type check them

---------------------------------------------
-- Attempt using 'data' to represent atoms --
---------------------------------------------

data TV = NullTV
        | SimpleTV Float Float
        | TVAndLink TV TV
        | TVOrLink TV TV
        | TVEvaluationLink EvaluationLink
        | TVGetTVLink Atom

data Atom = APredicate Predicate
          | AConcept Concept
          | ANumber Float

data Concept = Concept String
             | CAndLink Concept Concept
             | COrLink Concept Concept

data Predicate = Predicate ([Atom] -> TV)
               | PAndLink Predicate Predicate
               | POrLink Predicate Predicate

data EvaluationLink = EvaluationLink Predicate [Atom]

data SchemaLink = SchemaLink ([Atom] -> Atom)

data GetTVLink = GetTVLink Atom

-------------
-- Example --
-------------

-- Functions from [Atom] to TV to build predicates
is_bottom :: [Atom] -> TV
is_bottom _ = SimpleTV 0 0.9
is_top :: [Atom] -> TV
is_top _ = SimpleTV 1 0.9
is_car :: [Atom] -> TV
is_car [AConcept (Concept "BMW")] = SimpleTV 1 0.9
is_car [AConcept (Concept "Camel")] = SimpleTV 0.1 0.9
is_car _ = SimpleTV 0 0.9

-- And/Or hypergraph of concepts
h1 = COrLink (CAndLink (Concept "A") (Concept "B")) (Concept "C")

-- And/Or hypergraph of predicates
h2 = POrLink (PAndLink (Predicate is_top) (Predicate is_car)) (Predicate is_bottom)

-- Apply EvaluationLink to predicate h2
tv3 = TVEvaluationLink (EvaluationLink h2 [AConcept (Concept "BMW")])

-- Build a SchemaLink
add :: [Atom] -> Atom
add [ANumber x, ANumber y] = ANumber (x + y)
add _ = AConcept (Concept "undefined")
h4 = SchemaLink add

-- Test GetTVLink
tv1 = TVGetTVLink (AConcept h1)
tv2 = TVGetTVLink (APredicate h2)

-- Test AndLink with TV
tv4 = TVAndLink tv1 tv3
