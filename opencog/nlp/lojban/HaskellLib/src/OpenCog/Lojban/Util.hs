{-# LANGUAGE PatternSynonyms    #-}
module OpenCog.Lojban.Util where

import OpenCog.AtomSpace
import Data.List (nub, partition)

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
pattern PN name <-Node "PredicateNode" name _
pattern GPN name <-Node "GroundedPredicateNode" name _
pattern VN name <-Node "VariableNode" name _

pattern AL l <- Link "AndLink" l _
pattern LL l <- Link "ListLink" l _
pattern NL l <- Link "NotLink" l _
pattern ImpL l tv <- Link "ImplicationLink" l tv
pattern InhL l tv <- Link "InheritanceLink" l tv
pattern SL l <- Link "SetLink" l _
pattern SSL l <- Link "SatisfyingSetLink" l _
pattern EvalL tv p a <- Link "EvaluationLink" [p,a] tv
pattern ExL tv p a <- Link "ExistsLink" [p,a] tv
pattern CtxL c a <- Link "ContextLink" [c,a] _
pattern SimL a b <- Link "SimilarityLink" [a,b] _
pattern SubL a b <- Link "SubSetLink" [a,b] _
pattern LambdaL a b <- Link "LambdaLink" [a,b] _
pattern MemL a b <- Link "MemberLink" [a,b] _
pattern EquivL a b <- Link "EquivalenceLink" [a,b] _

cCN name tv = Node "ConceptNode" name tv
cPN name tv = Node "PredicateNode" name tv
cGPN name tv = Node "GroundedPredicateNode" name tv
cVN name    = Node "VariableNode" name noTv
cAN name    = Node "AnchorNode" name noTv
cNN name    = Node "NumberNode" name noTv
cTN name    = Node "TypeNode" name noTv

cLL a           = Link "ListLink"                             a     noTv
cSL a           = Link "SetLink"                              a     noTv
cSimL a b       = Link "SimilarityLink"                   [a,b]    noTv
cVL a           = Link "VariableList"                         a     noTv
cInhL tv a b    = Link "InheritanceLink"                  [a,b]     tv
cImpL tv a b    = Link "ImplicationLink"                  [a,b]     tv
cIFaoIFL tv a b = Link "AndLink"          [cImpL tv a b,cImpL tv b a] tv
cEvalL tv a b   = Link "EvaluationLink"                   [a,b]     tv
cSSL tv a       = Link "SatisfyingSetLink"                    a     tv
cExL tv a b     = Link "ExistsLink"                       [a,b]     tv
cFAL tv a b     = Link "ForAllLink"                       [a,b]     tv
cPL     a b     = Link "PutLink"                          [a,b]     noTv
cGL     a       = Link "GetLink"                            [a]     noTv
cAL  tv a       = Link "AndLink"                              a     tv
cOL  tv a       = Link "OrLink"                                   a tv
cNL  tv a       = Link "NotLink"                                [a] tv
cCtxL tv a b    = Link "ContextLink"                      [a,b]     tv
cLamdaL tv a b  = Link "LambdaLink"                       [a,b]     tv
cMemL tv a b    = Link "MemberLink"                       [a,b]     tv


-- Creates the abstract state for NU:
  -- Extracts predicate instances from atom and state, replaces them with VNs,
    -- and adds "is_event" atom
getKaState eventType atom state=
  let predicateNodes = nub $ (atomFold getPredicateNode [] atom) ++ (foldl getStatePredicateNode [] state) -- 1
      predicateVars = map (cVN.("$"++).show) [3..(length predicateNodes) + 2] -- 2
      state' = map (atomMap (replacePredicates (zip predicateNodes predicateVars))) (atom:state)
  in case eventType of
    Nothing -> (predicateVars, state')
    Just a  -> let eventAtom' = atomMap (makeEvents a (cCN "$2" highTv)) (head state')
                   -- removes duplicates caused by multiple sumti
                   eventAtom = atomMap (\a -> case a of (Link t ls tv) -> Link t (nub ls) tv
                                                        n              -> n) eventAtom'
               in (predicateVars, eventAtom:state')
  where
    -- Extracts PNs from an Atom with Evaluation links following the sumti for predicate structure
    -- To be used with atomFold
    getPredicateNode :: [Atom] -> Atom -> [Atom]
    getPredicateNode ns (EvalL _ _ (LL (pn@(PN _):_))) = pn:ns
    getPredicateNode ns _ = ns
    -- Extractes specific instance PNs from state
    getStatePredicateNode :: [Atom] -> Atom -> [Atom]
    getStatePredicateNode ns (ImpL [pn@(PN _), (PN _)] _) = pn:ns
    getStatePredicateNode ns (InhL [pn@(PN _), (PN _)] _) = pn:ns
    getStatePredicateNode ns _ = ns
    -- Replaces PNs in map with VarN
    replacePredicates :: [(Atom, Atom)] -> Atom -> Atom
    replacePredicates m pn@(PN _) = case lookup pn m of
      Just vn -> vn
      Nothing -> pn
    replacePredicates _ a = a
    makeEvents :: (TruthVal -> Atom -> Atom -> Atom) -> Atom -> Atom -> Atom
    makeEvents makeEvent cn (EvalL tv s@(PN _) (LL [vn@(VN _), (CN _)])) =
      makeEvent tv cn vn
    makeEvents _ _ a = a

getNuState eventType atom state=
  let predicateNodes = nub $ (atomFold getPredicateNode [] atom) ++ (foldl getStatePredicateNode [] state) -- 1
      predicateVars = map (cVN.("$"++).show) [3..(length predicateNodes) + 2] -- 2
      state' = map (atomMap (replacePredicates (zip predicateNodes predicateVars))) (atom:state)
  in case eventType of
    Nothing -> (predicateVars, state')
    Just a  -> let eventAtom' = atomMap (makeEvents a (cCN "$2" highTv)) (head state')
                   -- removes duplicates caused by multiple sumti
                   eventAtom = atomMap (\a -> case a of (Link t ls tv) -> Link t (nub ls) tv
                                                        n              -> n) eventAtom'
               in (predicateVars, eventAtom:state')
  where
    -- Extracts PNs from an Atom with Evaluation links following the sumti for predicate structure
    -- To be used with atomFold
    getPredicateNode :: [Atom] -> Atom -> [Atom]
    getPredicateNode ns (EvalL _ _ (LL (pn@(PN _):_))) = pn:ns
    getPredicateNode ns _ = ns
    -- Extractes specific instance PNs from state
    getStatePredicateNode :: [Atom] -> Atom -> [Atom]
    getStatePredicateNode ns (ImpL [pn@(PN _), (PN _)] _) = pn:ns
    getStatePredicateNode ns (InhL [pn@(PN _), (PN _)] _) = pn:ns
    getStatePredicateNode ns _ = ns
    -- Replaces PNs in map with VarN
    replacePredicates :: [(Atom, Atom)] -> Atom -> Atom
    replacePredicates m pn@(PN _) = case lookup pn m of
      Just vn -> vn
      Nothing -> pn
    replacePredicates _ a = a
    makeEvents :: (TruthVal -> Atom -> Atom -> Atom) -> Atom -> Atom -> Atom
    makeEvents makeEvent cn (EvalL tv s@(PN _) (LL [vn@(VN _), (CN _)])) =
      makeEvent tv cn vn
    makeEvents _ _ a = a

mkPropPre pred atom name = Link "EquivalenceLink"
                [cLamdaL highTv
                    (cVN "1")
                    (cEvalL highTv
                        (pred)
                        (cLL [cVN "1"]))
                ,cLamdaL highTv
                    (cVN "2")
                    (cAL highTv
                        [(cEvalL highTv
                            (cPN "ckaji_sumit1" lowTv)
                            (cLL [cPN ("ckaji_" ++ name) lowTv,cVN "2"])
                        )
                        ,(cEvalL highTv
                            (cPN "ckaji_sumit2" lowTv)
                            (cLL [cPN ("ckaji_" ++ name) lowTv,atom])
                        )]
                    )
                ] highTv

pattern PropPred atom <- Link "EquivalenceLink"
                                [_
                                , Link "LambdaLink"
                                    [ _
                                    , Link "AndLink"
                                        [_
                                        , Link "EvaluationLink"
                                            [ _
                                            , Link "ListLink" [_,atom] _
                                            ] _
                                        ] _
                                    ] _
                                ] _

isInteger s = case reads s :: [(Integer, String)] of
  [(_, "")] -> True
  _         -> False

isDouble s = case reads s :: [(Double, String)] of
  [(_, "")] -> True
  _         -> False

isNumeric :: String -> Bool
isNumeric s = isInteger s || isDouble s
