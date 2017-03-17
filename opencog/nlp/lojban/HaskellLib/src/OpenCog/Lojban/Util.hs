{-# LANGUAGE PatternSynonyms    #-}
module OpenCog.Lojban.Util where

import OpenCog.AtomSpace
import Data.List (nub)

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
pattern ImpL l tv <- Link "ImplicationLink" l tv
pattern InhL l tv <- Link "InheritanceLink" l tv
pattern SL l <- Link "SetLink" l _
pattern SSL v l <- Link "SatisfyingSetLink" [v,l] _
pattern EvalL tv p a <- Link "EvaluationLink" [p,a] tv
pattern ExL tv p a <- Link "ExistsLink" [p,a] tv
pattern CtxL c a <- Link "ContextLink" [c,a] _
pattern SimL a b <- Link "SimilarityLink" [a,b] _
pattern SubL a b <- Link "SubSetLink" [a,b] _
pattern LambdaL a b <- Link "LambdaLink" [a,b] _
pattern MemL a b <- Link "MemberLink" [a,b] _

cCN name tv = Node "ConceptNode" name tv
cPN name tv = Node "PredicateNode" name tv
cGPN name tv = Node "GroundedPredicateNode" name tv
cVN name    = Node "VariableNode" name noTv
cAN name    = Node "AnchorNode" name noTv
cNN name    = Node "NumberNode" name noTv

cLL l           = Link "ListLink"                             l     noTv
cSL l           = Link "SetLink"                              l     noTv
cSimL a b       = Link "SimilarityLink"                   [a,b]    noTv
cVL a           = Link "VariableList"                         a     noTv
cInhL tv a b    = Link "InheritanceLink"                  [a,b]     tv
cImpL tv a b    = Link "ImplicationLink"                  [a,b]     tv
cIFaoIFL tv a b = Link "AndLink"          [cImpL tv a b,cImpL tv b a] tv
cEvalL tv a b   = Link "EvaluationLink"                   [a,b]     tv
cSSL tv a b     = Link "SatisfyingSetLink"                [a,b]     tv
cExL tv a b     = Link "ExistsLink"                       [a,b]     tv
cFAL tv a b     = Link "ForAllLink"                       [a,b]     tv
cPL     a b     = Link "PutLink"                          [a,b]     noTv
cGL     a       = Link "GetLink"                            [a]     noTv
cAL  tv l       = Link "AndLink"                              l     tv
cOL  tv a       = Link "OrLink"                               a     tv
cNL  tv a       = Link "NotLink"                            [a]     tv
cCtxL tv a b    = Link "ContextLink"                      [a,b]     tv
cLamdaL tv a b  = Link "LambdaLink"                       [a,b]     tv
cMemL tv a b    = Link "MemberLink"                       [a,b]     tv


-- Performs transformations to make atom into a specific abstract instance
-- (And cleans the state of unneeded ImplicationLinks)
mkNuLink pred (atom, state) =
  let var1 = "$1"
      var2 = "$2"
      predicateNodes = nub $ atomFold getPredicateNode [] atom -- gets PNs from bridi
      vars = map (("$"++).show) [3..(length predicateNodes) + 2] -- generates vars
      predicateVars = map (flip cPN highTv) vars
      predicateVarMap = zip predicateNodes predicateVars
      varAtom = atomMap (replacePredicates predicateVarMap) atom
      eventAtom = atomMap (makeEvents (cPN var2 highTv)) varAtom
      abstractPredicateNodes = foldl (getAbstractPredicateNode predicateNodes) [] state
      sumtiImplicationLinks = map makeImplicationLink (zip predicateVars abstractPredicateNodes)
      specificInstanceImplicationLinks = map makeImplicationLink (zip predicateVars (repeat (cPN "Specific Instance" highTv)))
      cleanedState = filter (filterImplicationPredicates predicateNodes) state
      link = Link "EquivalenceLink"
        [ cEvalL highTv
          (pred)
          (cLL [cVN var1])
        , cMemL highTv
            (cVN var1)
            (cSSL highTv
              (cVN var2)
              (cExL highTv
                (cVL (map cVN vars))
                (cAL highTv
                  ([(eventAtom)
                  , (varAtom)
                  ] ++ specificInstanceImplicationLinks
                    ++ sumtiImplicationLinks)
                )
              )
            )
          ] highTv
      in link:cleanedState
  where
    -- Use vars in varAtom to create "is_event" structure paralleling varAtom structure
    makeEvents :: Atom -> Atom -> Atom
    makeEvents cn (EvalL tv s@(PN _) (LL [vn@(PN _), (CN _)])) =
      cEvalL tv (cPN "is_event" highTv) (cLL [cn, vn])
    makeEvents _ a = a
    -- Extracts PredicateNode's from an Atom with Evaluation links following the sumti for predicate structure
    -- To be used with atomFold
    getPredicateNode :: [Atom] -> Atom -> [Atom]
    getPredicateNode ns (EvalL _ _ (LL (pn@(PN _):_))) = pn:ns
    getPredicateNode ns _ = ns
    -- Use extracted predicates to gather abstract versions from state
    getAbstractPredicateNode :: [Atom] -> [Atom] -> Atom -> [Atom]
    getAbstractPredicateNode pns ns (ImpL [pn1@(PN _), pn2@(PN _)] _) =
      if elem pn1 pns then pn2:ns else ns
    getAbstractPredicateNode _ ns _ = ns
    -- Replaces predicates in the map with the values in the abstract bridi
    -- Or does nothing
    replacePredicates :: [(Atom, Atom)] -> Atom -> Atom
    replacePredicates m (EvalL tv s@(PN _) (LL [pn@(PN _), cn@(CN _)])) =
      let vn = case lookup pn m of
            Just vn -> vn
            Nothing -> pn
      in cEvalL tv s (cLL [vn, cn])
    replacePredicates _ a = a
    -- create implication link given two atoms (could be Iso)
    makeImplicationLink :: (Atom, Atom) -> Atom
    makeImplicationLink (p1, p2) = cImpL highTv p1 p2
    -- Eliminate implication links of predicates (from state)
    filterImplicationPredicates :: [Atom] -> Atom -> Bool
    filterImplicationPredicates pns (ImpL [p1@(PN _), (PN _)] _) =
      if elem p1 pns then False else True
    filterImplicationPredicates _ _ = True

-- Inverse of the above
-- Extracts atom, instantiates vars, and adds ImplicationLinks to the state again
-- TODO: add random seed name once Roman finished ISO work
revNuLink :: [Atom] -> (Atom, [Atom])
revNuLink (link:cleanedState) =
  let predicateImplicationLinks = nub $ atomFold getImplicationLinks [] link
      atom = head $ atomFold cleanLink [] link
      -- Generate seed for specific gismu instances to replace vars with
  in (atom, predicateImplicationLinks ++ cleanedState)
  where
    -- Gets ImplicationLinks containign gismu PredicateNodes
    getImplicationLinks :: [Atom] -> Atom -> [Atom]
    getImplicationLinks ns i@(ImpL [(PN _), (PN name2)] _) =
      if name2 /= "Specific Instance" then ns else i:ns
    getImplicationLinks ns _ = ns
    -- Detect ImplicationLinks
    filterImplicationLinks :: Atom -> Bool
    filterImplicationLinks (ImpL _ _) = True
    filterImplicationLinks _ = False
    -- Traverses atom tree and flags an "is_event" PredicateNode
    hasEvent :: Atom -> Bool
    hasEvent a = atomFold isEvent False a
      where isEvent True _            = True
            isEvent _ (PN "is_event") = True
            isEvent _ _               = False
    -- Removes implicationLinks and "is_event" atoms
    -- Leaving only the atom
    cleanLink :: [Atom] -> Atom -> [Atom]
    cleanLink atom (AL l) =
      let noImplications = filter (not.filterImplicationLinks) l
          noEvents = filter (not.hasEvent) noImplications
      in noEvents
    cleanLink atom _ = atom

mkCtxPre pred atom =  Link "EquivalenceLink"
                        [cLamdaL highTv
                            (cVN "1")
                            (cEvalL highTv
                                (pred)
                                (cLL [cVN "1"]))
                        ,cLamdaL highTv
                            (cVN "2")
                            (cCtxL highTv
                                (cVN "2")
                                (atom))
                        ] highTv

pattern CtxPred atom <- Link "EquivalenceLink"
                                [ _
                                , Link "LambdaLink" [ _
                                                    ,Link "ContextLink" [ _
                                                                        , atom
                                                                        ] _
                                                    ] _
                                ] _


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
