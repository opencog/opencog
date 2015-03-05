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

data Atom = ANode Node | ALink Link

data Node = NPredicate Predicate
          | NConcept Concept
          | NNumber Float

data Link = LAndLink AndLink
          | LOrLink OrLink
          | LEvaluationLink EvaluationLink
          | LSchemaLink SchemaLink

data Concept = Concept String
             | CAndLink Concept Concept
             | COrLink Concept Concept

data Predicate = Predicate ([Atom] -> TV)
               | PAndLink Predicate Predicate
               | POrLink Predicate Predicate

data AndLink = AndLinkConcept Concept Concept
             | AndLinkPredicate Predicate Predicate
             | AndLinkTV TV TV

data OrLink = OrLinkConcept Concept Concept
            | OrLinkPredicate Predicate Predicate
            | OrLinkTV TV TV

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
is_car [ANode (NConcept (Concept "BMW"))] = SimpleTV 1 0.9
is_car [ANode (NConcept (Concept "Camel"))] = SimpleTV 0.1 0.9
is_car _ = SimpleTV 0 0.9

-- And/Or hypergraph of concepts
h1 = COrLink (CAndLink (Concept "A") (Concept "B")) (Concept "C")

-- And/Or hypergraph of predicates
h2 = POrLink (PAndLink (Predicate is_top) (Predicate is_car)) (Predicate is_bottom)

-- Apply EvaluationLink to predicate h2
tv3 = TVEvaluationLink (EvaluationLink h2 [ANode (NConcept (Concept "BMW"))])

-- Build a SchemaLink
add :: [Atom] -> Atom
add [ANode (NNumber x), ANode (NNumber y)] = ANode (NNumber (x + y))
add _ = ANode (NConcept (Concept "undefined"))
h4 = SchemaLink add

-- Test GetTVLink
tv1 = TVGetTVLink (ANode (NConcept h1))
tv2 = TVGetTVLink (ANode (NPredicate h2))

-- Test AndLink with TV
tv4 = TVAndLink tv1 tv3
