{-# LANGUAGE PatternSynonyms    #-}
module OpenCog.Lojban.Util where

import OpenCog.AtomSpace
import Control.Applicative
import Data.List (nub)

atomFind :: (Atom -> Bool) -> Atom -> Maybe Atom
atomFind p l@(Link t ls tv) = if p l
                                 then Just l
                                 else foldl (\ma a -> atomFind p a <|> ma) Nothing ls
atomFind p n@(Node _ _ _) = if p n
                               then Just n
                               else Nothing

atomAny :: (Atom -> Bool) -> Atom -> Bool
atomAny p l@(Link t ls tv) = p l || any (atomAny p) ls
atomAny p n@(Node _ _ _)   = p n

mapFst :: (a -> b) -> (a,c) -> (b,c)
mapFst f (a,c) = (f a,c)

mapSnd :: (a -> b) -> (c,a) -> (c,b)
mapSnd f (c,a) = (c,f a)

highTv :: TruthVal
highTv = stv 1 0.9

lowTv :: TruthVal
lowTv = stv 0.000001 0.01

if' :: Bool -> a -> a -> a
if' True a _ = a
if' False _ a = a

infixr 1 ?
(?) :: Bool -> a -> a -> a
(?) = if'

infixl 8 ...
(...) :: (c -> d) -> (a -> b -> c) -> a -> b -> d
(...) = (.).(.)

pattern CN name <-Node "ConceptNode" name _
pattern AN name <-Node "AnchorNode" name _
pattern PN name <-Node "PredicateNode" name _
pattern GPN name <-Node "GroundedPredicateNode" name _
pattern VN name <-Node "VariableNode" name _
pattern DSN name <- Node "DefinedSchemaNode" name _

pattern AL l <- Link "AndLink" l _
pattern LL l <- Link "ListLink" l _
pattern NL l <- Link "NotLink" l _
pattern DL l <- Link "DefineLink" l _
pattern ImpL tv a b <- Link "ImplicationLink" [a,b] tv
pattern InhL tv a b <- Link "InheritanceLink" [a,b] tv
pattern SL l <- Link "SetLink" l _
pattern SSScL l <- Link "SatisfyingSetScopeLink" l _
pattern EvalL tv p a <- Link "EvaluationLink" [p,a] tv
pattern ExL tv p a <- Link "ExistsLink" [p,a] tv
pattern CtxL c a <- Link "ContextLink" [c,a] _
pattern SimL a b <- Link "SimilarityLink" [a,b] _
pattern SubL a b <- Link "SubsetLink" [a,b] _
pattern LambdaL a b <- Link "LambdaLink" [a,b] _
pattern MemL a b <- Link "MemberLink" [a,b] _
pattern EquivL a b <- Link "EquivalenceLink" [a,b] _
pattern EXOL a <- Link "ExecutionOutputLink" a _

cCN name tv = Node "ConceptNode" name tv
cPN name tv = Node "PredicateNode" name tv
cGPN name tv = Node "GroundedPredicateNode" name tv
cVN name    = Node "VariableNode" name noTv
cAN name    = Node "AnchorNode" name noTv
cNN name    = Node "NumberNode" name noTv
cTN name    = Node "TypeNode" name noTv
cDSN name  = Node "DefinedSchemaNode" name noTv

cLL a           = Link "ListLink"                             a     noTv
cSL a           = Link "SetLink"                              a     noTv
cSimL a b       = Link "SimilarityLink"                   [a,b]    noTv
cVL a           = Link "VariableList"                         a     noTv
cInhL tv a b    = Link "InheritanceLink"                  [a,b]     tv
cImpL tv a b    = Link "ImplicationLink"                  [a,b]     tv
cIImpL tv a b   = Link "IntensionalImplicationLink"       [a,b]     tv
cIFaoIFL tv a b = Link "AndLink"          [cImpL tv a b,cImpL tv b a] tv
cEvalL tv a b   = Link "EvaluationLink"                   [a,b]     tv
cSSScL tv a     = Link "SatisfyingSetScopeLink"               a     tv
cExL tv a b     = Link "ExistsLink"                       [a,b]     tv
cFAL tv a b     = Link "ForAllLink"                       [a,b]     tv
cPL     a b     = Link "PutLink"                          [a,b]     noTv
cGL     a       = Link "GetLink"                            [a]     noTv
cAL  tv a       = Link "AndLink"                              a     tv
cOL  tv a       = Link "OrLink"                               a     tv
cNL  tv a       = Link "NotLink"                            [a]     tv
cDL  tv a       = Link "DefineLink"                           a     tv
cEXOL tv a      = Link "ExecutionOutputLink"                  a     tv
cEXL tv a b     = Link "ExecutionOutputLink"              [a,b]     tv
cCtxL tv a b    = Link "ContextLink"                      [a,b]     tv
cLamdaL tv a b  = Link "LambdaLink"                       [a,b]     tv
cMemL tv a b    = Link "MemberLink"                       [a,b]     tv
cEquivL tv a b  = Link "EquivalenceLink"                  [a,b]     tv
cSubL tv a b    = Link "SubsetLink"                       [a,b]     tv
cTVL a b        = Link "TypedVariableLink"                [a,b]     noTv
cTCL ls         = Link "TypedChoice"                         ls     noTv

isInteger s = case reads s :: [(Integer, String)] of
  [(_, "")] -> True
  _         -> False

isDouble s = case reads s :: [(Double, String)] of
  [(_, "")] -> True
  _         -> False

isNumeric :: String -> Bool
isNumeric s = isInteger s || isDouble s
